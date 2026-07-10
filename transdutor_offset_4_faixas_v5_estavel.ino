#include <Arduino.h>
#include "HX711.h"

// =====================================================
// PINOS ESP8266 / NODEMCU
// =====================================================
#define HX711_DT  D5
#define HX711_SCK D6

HX711 scale;

// =====================================================
// CONFIGURACAO DO TRANSDUTOR
// =====================================================
const float SENSIBILIDADE_MV_V = 2.0f;
long referenciaADC = 45632L;

// No seu conjunto: 21800 counts correspondem a 1% do fundo de escala.
const float ADC_POR_1_PERCENT_FS = 21800.0f;

// =====================================================
// LIMITES DE OFFSET (mV/V)
// =====================================================
const float LIMITE_EXCELENTE = 0.03f;
const float LIMITE_APROVADO  = 0.08f;
const float LIMITE_ATENCAO   = 0.15f;

// =====================================================
// DETECCAO DE TRANSDUTOR DESCONECTADO
// =====================================================
const long ADC_ENTRADA_ABERTA_CENTRO = 3100L;
const long TOLERANCIA_ENTRADA_ABERTA = 6000L;
const long VARIACAO_MAX_ENTRADA_ABERTA = 5000L;

const long LIMITE_SATURACAO_ADC = 7800000L;
const long VARIACAO_MAXIMA_CONECTADO = 80000L;
const long SALTO_MAXIMO_ENTRE_LEITURAS = 350000L;

const uint8_t FALHAS_PARA_DESCONECTAR = 2;
const uint8_t BONS_PARA_RECONECTAR = 3;

// =====================================================
// AJUSTES DE VELOCIDADE
// =====================================================
// Atualizacao limitada a aproximadamente 8 vezes por segundo.
// As conversoes continuam sendo lidas assim que ficam prontas, mas o dashboard
// recebe dados em intervalos de 120 ms para melhorar a estabilidade visual.
const unsigned long INTERVALO_MINIMO_ENVIO_MS = 120;

// So acusa erro se o HX711 ficar este tempo inteiro sem entregar leitura.
const unsigned long TIMEOUT_SEM_LEITURA_MS = 1200;

// Janela curta usada para diagnostico e filtragem, sem bloquear novas leituras.
const uint8_t JANELA_DIAGNOSTICO = 8;

// Filtro exponencial usado somente para a exibicao no dashboard.
// Quanto menor o ALPHA, mais suave fica a leitura visual.
const float EMA_ALPHA = 0.22f;

long amostrasDiagnostico[JANELA_DIAGNOSTICO] = {0};
uint8_t indiceDiagnostico = 0;
uint8_t quantidadeDiagnostico = 0;

float emaDiferenca = 0.0f;
bool emaInicializado = false;

unsigned long ultimaLeituraMs = 0;
unsigned long ultimoEnvioMs = 0;
bool erroHX711Enviado = false;

long leituraAnterior = 0;
bool possuiLeituraAnterior = false;
bool transdutorConectado = false;
uint8_t contadorFalhas = 0;
uint8_t contadorBons = 0;

struct DiagnosticoLeitura {
  long media;
  long minimo;
  long maximo;
  long variacao;
  long salto;
  bool hx711Ok;
  bool saturado;
  bool instavel;
  bool saltoExcessivo;
  bool entradaAberta;
  bool valido;
};

float converterAdcParaPercentualFS(long diferencaADC) {
  return (float)diferencaADC / ADC_POR_1_PERCENT_FS;
}

float converterPercentualParaMvV(float percentualFS) {
  return (percentualFS / 100.0f) * SENSIBILIDADE_MV_V;
}

const char* classificarOffset(float offsetAbsolutoMvV) {
  if (offsetAbsolutoMvV <= LIMITE_EXCELENTE) return "EXCELENTE";
  if (offsetAbsolutoMvV <= LIMITE_APROVADO)  return "APROVADO";
  if (offsetAbsolutoMvV <= LIMITE_ATENCAO)   return "ATENCAO";
  return "REPROVADO";
}

void adicionarAmostraDiagnostico(long valor) {
  amostrasDiagnostico[indiceDiagnostico] = valor;
  indiceDiagnostico = (indiceDiagnostico + 1) % JANELA_DIAGNOSTICO;
  if (quantidadeDiagnostico < JANELA_DIAGNOSTICO) quantidadeDiagnostico++;
}

long calcularFiltroEMA(long valor) {
  if (!emaInicializado) {
    emaDiferenca = (float)valor;
    emaInicializado = true;
  } else {
    emaDiferenca = (EMA_ALPHA * (float)valor) +
                   ((1.0f - EMA_ALPHA) * emaDiferenca);
  }
  return (long)lroundf(emaDiferenca);
}

DiagnosticoLeitura diagnosticar(long leituraAtual) {
  DiagnosticoLeitura d = {};
  d.hx711Ok = true;
  d.minimo = 8388607L;
  d.maximo = -8388608L;

  adicionarAmostraDiagnostico(leituraAtual);

  int64_t soma = 0;
  for (uint8_t i = 0; i < quantidadeDiagnostico; i++) {
    const long valor = amostrasDiagnostico[i];
    soma += valor;
    if (valor < d.minimo) d.minimo = valor;
    if (valor > d.maximo) d.maximo = valor;
  }

  d.media = (long)(soma / quantidadeDiagnostico);
  d.variacao = d.maximo - d.minimo;
  d.salto = possuiLeituraAnterior ? labs(leituraAtual - leituraAnterior) : 0;

  d.saturado = labs(d.minimo) >= LIMITE_SATURACAO_ADC ||
               labs(d.maximo) >= LIMITE_SATURACAO_ADC;

  // A instabilidade so e avaliada depois de preencher a janela.
  d.instavel = quantidadeDiagnostico >= JANELA_DIAGNOSTICO &&
               d.variacao > VARIACAO_MAXIMA_CONECTADO;

  d.saltoExcessivo = possuiLeituraAnterior &&
                     d.salto > SALTO_MAXIMO_ENTRE_LEITURAS;

  const bool pertoDaAssinaturaAberta =
    labs(d.media - ADC_ENTRADA_ABERTA_CENTRO) <= TOLERANCIA_ENTRADA_ABERTA;

  const bool baixaVariacao =
    quantidadeDiagnostico >= 2 &&
    d.variacao <= VARIACAO_MAX_ENTRADA_ABERTA;

  d.entradaAberta = pertoDaAssinaturaAberta && baixaVariacao;

  d.valido = !d.saturado && !d.instavel &&
             !d.saltoExcessivo && !d.entradaAberta;

  leituraAnterior = leituraAtual;
  possuiLeituraAnterior = true;
  return d;
}

void atualizarEstadoConexao(const DiagnosticoLeitura& d) {
  if (d.valido) {
    contadorFalhas = 0;
    if (contadorBons < BONS_PARA_RECONECTAR) contadorBons++;
    if (contadorBons >= BONS_PARA_RECONECTAR) transdutorConectado = true;
  } else {
    contadorBons = 0;
    if (contadorFalhas < FALHAS_PARA_DESCONECTAR) contadorFalhas++;
    if (contadorFalhas >= FALHAS_PARA_DESCONECTAR) transdutorConectado = false;
  }
}

const char* motivoDesconexao(const DiagnosticoLeitura& d) {
  if (d.entradaAberta) return "ENTRADA_ABERTA";
  if (d.saturado) return "SATURACAO_ADC";
  if (d.instavel) return "SINAL_INSTAVEL";
  if (d.saltoExcessivo) return "SALTO_EXCESSIVO";
  if (!transdutorConectado) return "VALIDANDO_CONEXAO";
  return "OK";
}

void enviarErroHX711() {
  Serial.println("{\"status\":\"ERRO_HX711\",\"conectado\":false,\"motivo\":\"SEM_RESPOSTA_HX711\"}");
}

void enviarJson(const DiagnosticoLeitura& d) {
  const long diferenca = d.media - referenciaADC;
  const long filtrado = calcularFiltroEMA(diferenca);
  const float percentualFS = converterAdcParaPercentualFS(diferenca);
  const float offsetMvV = converterPercentualParaMvV(percentualFS);
  const float offsetAbsolutoMvV = fabs(offsetMvV);
  const float tensaoEntradaMv = offsetMvV * 3.3f;

  // A classificacao usa a diferenca de diagnostico, sem o filtro EMA.
  // O campo "filtrado" serve apenas para deixar o dashboard mais suave.
  const char* status = transdutorConectado
    ? classificarOffset(offsetAbsolutoMvV)
    : "DESCONECTADO";

  Serial.print("{\"bruto\":");
  Serial.print(d.media);
  Serial.print(",\"referenciaADC\":");
  Serial.print(referenciaADC);
  Serial.print(",\"diferenca\":");
  Serial.print(diferenca);
  Serial.print(",\"filtrado\":");
  Serial.print(filtrado);
  Serial.print(",\"porcentagemFS\":");
  Serial.print(percentualFS, 4);
  Serial.print(",\"offset_mV_V\":");
  Serial.print(offsetMvV, 5);
  Serial.print(",\"offsetAbs_mV_V\":");
  Serial.print(offsetAbsolutoMvV, 5);
  Serial.print(",\"tensao_mV\":");
  Serial.print(tensaoEntradaMv, 6);
  Serial.print(",\"variacaoADC\":");
  Serial.print(d.variacao);
  Serial.print(",\"saltoADC\":");
  Serial.print(d.salto);
  Serial.print(",\"saturado\":");
  Serial.print(d.saturado ? "true" : "false");
  Serial.print(",\"instavel\":");
  Serial.print(d.instavel ? "true" : "false");
  Serial.print(",\"entradaAberta\":");
  Serial.print(d.entradaAberta ? "true" : "false");
  Serial.print(",\"conectado\":");
  Serial.print(transdutorConectado ? "true" : "false");
  Serial.print(",\"motivo\":\"");
  Serial.print(motivoDesconexao(d));
  Serial.print("\",\"limiteExcelente\":");
  Serial.print(LIMITE_EXCELENTE, 2);
  Serial.print(",\"limiteAprovado\":");
  Serial.print(LIMITE_APROVADO, 2);
  Serial.print(",\"limiteAtencao\":");
  Serial.print(LIMITE_ATENCAO, 2);
  Serial.print(",\"filtroEMAAlpha\":");
  Serial.print(EMA_ALPHA, 2);
  Serial.print(",\"intervaloEnvioMs\":");
  Serial.print(INTERVALO_MINIMO_ENVIO_MS);
  Serial.print(",\"status\":\"");
  Serial.print(status);
  Serial.println("\"}");
}

void limparJanelas() {
  quantidadeDiagnostico = 0;
  indiceDiagnostico = 0;
  emaDiferenca = 0.0f;
  emaInicializado = false;
  possuiLeituraAnterior = false;
}

void capturarNovaReferencia() {
  // A referencia usa 10 leituras para ficar estavel. Esse comando pode levar
  // cerca de 1 segundo em HX711 configurado para 10 SPS.
  const uint8_t quantidade = 10;
  int64_t soma = 0;

  for (uint8_t i = 0; i < quantidade; i++) {
    const unsigned long inicio = millis();
    while (!scale.is_ready()) {
      if (millis() - inicio > TIMEOUT_SEM_LEITURA_MS) {
        Serial.println("{\"status\":\"REFERENCIA_RECUSADA\",\"motivo\":\"SEM_RESPOSTA_HX711\"}");
        return;
      }
      delay(1);
      yield();
    }
    soma += scale.read();
  }

  const long candidata = (long)(soma / quantidade);
  DiagnosticoLeitura d = diagnosticar(candidata);
  atualizarEstadoConexao(d);

  if (!d.valido || !transdutorConectado) {
    Serial.print("{\"status\":\"REFERENCIA_RECUSADA\",\"motivo\":\"");
    Serial.print(motivoDesconexao(d));
    Serial.println("\"}");
    return;
  }

  referenciaADC = candidata;
  limparJanelas();
  Serial.print("{\"status\":\"NOVA_REFERENCIA\",\"referenciaADC\":");
  Serial.print(referenciaADC);
  Serial.println("}");
}

void processarComandoSerial() {
  if (!Serial.available()) return;
  const char comando = Serial.read();
  if (comando == 'r' || comando == 'R') capturarNovaReferencia();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  scale.begin(HX711_DT, HX711_SCK);
  ultimaLeituraMs = millis();

  Serial.print("{\"status\":\"PRONTO\",\"referenciaADC\":");
  Serial.print(referenciaADC);
  Serial.println(",\"modo\":\"V5_ESTAVEL_EMA_120MS\"}");
}

void loop() {
  processarComandoSerial();

  if (scale.is_ready()) {
    const long leituraAtual = scale.read();
    ultimaLeituraMs = millis();
    erroHX711Enviado = false;

    DiagnosticoLeitura d = diagnosticar(leituraAtual);
    atualizarEstadoConexao(d);

    if (millis() - ultimoEnvioMs >= INTERVALO_MINIMO_ENVIO_MS) {
      ultimoEnvioMs = millis();
      enviarJson(d);
    }
  } else {
    // Nao gera erro entre duas conversoes normais. So acusa quando o HX711
    // permanece sem responder por tempo prolongado.
    if (millis() - ultimaLeituraMs >= TIMEOUT_SEM_LEITURA_MS && !erroHX711Enviado) {
      transdutorConectado = false;
      contadorBons = 0;
      enviarErroHX711();
      erroHX711Enviado = true;
    }
  }

  yield();
}

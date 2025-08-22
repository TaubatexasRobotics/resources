#include <FastLED.h>
#define LED_PIN     7
#define NUM_LEDS    30

CRGB leds[NUM_LEDS];

void setup() {
  Serial.begin(9600); // Inicia a comunicação serial com baud rate de 9600
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  // Inicializa os LEDs com uma cor padrão (opcional)
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 0, 0); // Cor inicial: vermelho
  }
  FastLED.show();
}

void loop() {
  if (Serial.available() > 0) { // Verifica se há dados na serial
    byte received = Serial.read(); // Lê o byte recebido
    if (received == 'r') { // Verifica se o byte é 0x12
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(255, 0, 0); // Muda para azul (ou a cor desejada)
      }
      FastLED.show(); // Atualiza os LEDs
    }
    if (received == 'g') { // Verifica se o byte é 0x12
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 255, 0); // Muda para azul (ou a cor desejada)
      }
      FastLED.show(); // Atualiza os LEDs
    }
    if (received == 'b') { // Verifica se o byte é 0x12
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = CRGB(0, 0, 255); // Muda para azul (ou a cor desejada)
      }
      FastLED.show(); // Atualiza os LEDs
    }
  }
}

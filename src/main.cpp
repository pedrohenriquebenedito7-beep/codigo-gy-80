#include <Arduino.h>
#include <Wire.h>
#include <cmath> // std::abs seguro para float

// Pinos e endereço definidos com tipagem segura (C++)
constexpr uint8_t SDA_PIN = 21;
constexpr uint8_t SCL_PIN = 22;
constexpr uint8_t L3G4200D_ADDR = 0x69;

// Registradores organizados em um namespace
namespace Reg {
  constexpr uint8_t WHO_AM_I  = 0x0F;
  constexpr uint8_t CTRL_REG1 = 0x20;
  constexpr uint8_t CTRL_REG4 = 0x23;
  constexpr uint8_t OUT_X_L   = 0x28;
}



// Struct para agrupar os 3 eixos em vez de criar 9 variáveis soltas
struct Eixos {
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
};

Eixos offset;
Eixos angulo;
unsigned long lastTime = 0;

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(L3G4200D_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(L3G4200D_ADDR);                      // Inicia conexão com o sensor giroscópio
  Wire.write(reg);                                            // Busca o endereço de memória 
  Wire.endTransmission(false);                                // Mantem a comunicação ativa com o canal 
  Wire.requestFrom(L3G4200D_ADDR, static_cast<uint8_t>(1));   // Passa para o 'modo escutar' e recebe 1 byte do canal
  return Wire.available() ? Wire.read() : 0;                  // confere se tem o byte na porta do esp (se sim '?' retorna o byte) Wire.requestFrom(endereço, quantidade)
}                                                             // se não retorna zero ':'

void calibrarGiroscopio() {
  constexpr int amostras = 300;
  long somaX = 0, somaY = 0, somaZ = 0;

  for (int i = 0; i < amostras; i++) {
    Wire.beginTransmission(L3G4200D_ADDR);  // inicia a comunicação com o sensor
    Wire.write(Reg::OUT_X_L | 0x80);        // 'Varre' as portas a partir da 0x28 = OUT_X_L (tipo um for)
    Wire.endTransmission(false);            // Não encerra a comunicação

    if (Wire.requestFrom(L3G4200D_ADDR, static_cast<uint8_t>(6)) == 6) {

      uint8_t xL = Wire.read(); uint8_t xH = Wire.read();
      uint8_t yL = Wire.read(); uint8_t yH = Wire.read();
      uint8_t zL = Wire.read(); uint8_t zH = Wire.read();

      somaX += static_cast<int16_t>((xH << 8) | xL);
      somaY += static_cast<int16_t>((yH << 8) | yL);
      somaZ += static_cast<int16_t>((zH << 8) | zL);

      // São 6 bytes de comunicação por que são 2 bytes de cada eixo
      // 1 byte high e 1 byte low 
      // basicamente essa diferença aumenta a precisao do sensor, que lê até 0.00875 graus/s
      // O byte do low chega a 11111111 adiciona-se 1 bit no byte do high (00000001 00000000)
    }
    delay(5);
  }

  offset.x = static_cast<float>(somaX) / amostras;
  offset.y = static_cast<float>(somaY) / amostras;
  offset.z = static_cast<float>(somaZ) / amostras;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t id = readRegister(Reg::WHO_AM_I);
  if (id != 0xD3) {
    Serial.print("Sensor nao encontrado. ID lido: 0x");
    Serial.println(id, HEX);
    while (true) delay(100);
  }

  writeRegister(Reg::CTRL_REG1, 0x0F);
  writeRegister(Reg::CTRL_REG4, 0x80);
  delay(100);

  calibrarGiroscopio();
  lastTime = micros();
}

void loop() {
  Wire.beginTransmission(L3G4200D_ADDR);
  Wire.write(Reg::OUT_X_L | 0x80);
  Wire.endTransmission(false);

  if (Wire.requestFrom(L3G4200D_ADDR, static_cast<uint8_t>(6)) == 6) { // Wire.requestFrom essa funcao age pedindo a mensagem e retorna no codigo o tamanho de bytes
    uint8_t xL = Wire.read(); uint8_t xH = Wire.read();
    uint8_t yL = Wire.read(); uint8_t yH = Wire.read();
    uint8_t zL = Wire.read(); uint8_t zH = Wire.read();

    int16_t rawX = static_cast<int16_t>((xH << 8) | xL); // a funcao static_cast considera tambem a parte negativa dos bits
    int16_t rawY = static_cast<int16_t>((yH << 8) | yL); // ((yH << 8) | yL) essa parte soma yH + yL, mas yH é considerado apenas a partri do oitavo
    int16_t rawZ = static_cast<int16_t>((zH << 8) | zL);

    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0f;  // calculo do dt pra integrar 
    lastTime = now;

    Eixos vel;
    vel.x = (rawX - offset.x) * 0.00875f; // retira o offset ou seja o valor calibrado que o sensor atua flutuando 
    vel.y = (rawY - offset.y) * 0.00875f;
    vel.z = (rawZ - offset.z) * 0.00875f;

    if (std::abs(vel.x) < 0.5f) vel.x = 0.0f;
    if (std::abs(vel.y) < 0.5f) vel.y = 0.0f;
    if (std::abs(vel.z) < 0.5f) vel.z = 0.0f;

    angulo.x += vel.x * dt;
    angulo.y += vel.y * dt;
    angulo.z += vel.z * dt;

    Serial.print(">Taxa de Rolagem:"); Serial.println(vel.x, 2);
    Serial.print(">Taxa de Arfagem:"); Serial.println(vel.y, 2);
    Serial.print(">Taxa de Guinada:"); Serial.println(vel.z, 2);
    Serial.print(">Rolagem:"); Serial.println(angulo.x, 2);
    Serial.print(">Arfagem:"); Serial.println(angulo.y, 2);
    Serial.print(">Guinada:"); Serial.println(angulo.z, 2);

    Serial.println(">RefVelMax:15|np");
    Serial.println(">RefVelMin:-15|np");
  }

  delay(10);
}
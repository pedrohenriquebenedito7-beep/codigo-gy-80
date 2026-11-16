#include <Wire.h>

#define SDA_PIN 21
#define SCL_PIN 22
#define L3G4200D_ADDR 0x69

#define WHO_AM_I   0x0F
#define CTRL_REG1  0x20
#define CTRL_REG4  0x23
#define OUT_X_L    0x28

float offsetX = 0, offsetY = 0, offsetZ = 0;
float angX = 0, angY = 0, angZ = 0;
unsigned long lastTime = 0;

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(L3G4200D_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg) {
  Wire.beginTransmission(L3G4200D_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)L3G4200D_ADDR, (uint8_t)1);
  return Wire.available() ? Wire.read() : 0;
}

void calibrarGiroscopio() {
  const int amostras = 300;
  long somaX = 0, somaY = 0, somaZ = 0;

  for (int i = 0; i < amostras; i++) {
    Wire.beginTransmission(L3G4200D_ADDR);
    Wire.write(OUT_X_L | 0x80);
    Wire.endTransmission(false);

    if (Wire.requestFrom((uint8_t)L3G4200D_ADDR, (uint8_t)6) == 6) {
      uint8_t xL = Wire.read(); uint8_t xH = Wire.read();
      uint8_t yL = Wire.read(); uint8_t yH = Wire.read();
      uint8_t zL = Wire.read(); uint8_t zH = Wire.read();

      somaX += (int16_t)((xH << 8) | xL);
      somaY += (int16_t)((yH << 8) | yL);
      somaZ += (int16_t)((zH << 8) | zL);
    }
    delay(5);
  }

  offsetX = (float)somaX / amostras;
  offsetY = (float)somaY / amostras;
  offsetZ = (float)somaZ / amostras;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);

  uint8_t id = readRegister(WHO_AM_I);
  if (id != 0xD3) {
    Serial.print("Sensor nao encontrado. ID lido: 0x");
    Serial.println(id, HEX);
    while (1) delay(100);
  }

  writeRegister(CTRL_REG1, 0x0F); // 100 Hz, eixos XYZ ativos
  writeRegister(CTRL_REG4, 0x80); // BDU ativo, escala 250 dps
  delay(100);

  calibrarGiroscopio();
  lastTime = micros();
}

void loop() {
  Wire.beginTransmission(L3G4200D_ADDR);
  Wire.write(OUT_X_L | 0x80);
  Wire.endTransmission(false);

  if (Wire.requestFrom((uint8_t)L3G4200D_ADDR, (uint8_t)6) == 6) {
    uint8_t xL = Wire.read(); uint8_t xH = Wire.read();
    uint8_t yL = Wire.read(); uint8_t yH = Wire.read();
    uint8_t zL = Wire.read(); uint8_t zH = Wire.read();

    int16_t rawX = (int16_t)((xH << 8) | xL);
    int16_t rawY = (int16_t)((yH << 8) | yL);
    int16_t rawZ = (int16_t)((zH << 8) | zL);

    // Calcula o dt exato em segundos
    unsigned long now = micros();
    float dt = (now - lastTime) / 1000000.0;
    lastTime = now;

    // Velocidade angular (°/s) descontando o erro estático
    float velX = (rawX - offsetX) * 0.00875;
    float velY = (rawY - offsetY) * 0.00875;
    float velZ = (rawZ - offsetZ) * 0.00875;

    // Zona morta pequena para evitar acúmulo de ruído parado
    if (abs(velX) < 0.35) velX = 0;
    if (abs(velY) < 0.35) velY = 0;
    if (abs(velZ) < 0.35) velZ = 0;

    // Integração numérica -> Posição angular (°)
    angX += velX * dt;
    angY += velY * dt;
    angZ += velZ * dt;

    // Plotagem com escala fixa (-180 a 180)
  Serial.print(">VelX:"); Serial.println(velX, 2);
  Serial.print(">VelY:"); Serial.println(velY, 2);
  Serial.print(">VelZ:"); Serial.println(velZ, 2);
  Serial.print(">AngX:"); Serial.println(angX, 2);
  Serial.print(">AngY:"); Serial.println(angY, 2);
  Serial.print(">AngZ:"); Serial.println(angZ, 2);
  }

  delay(10); // Mantém a taxa próxima aos 100 Hz do sensor
}
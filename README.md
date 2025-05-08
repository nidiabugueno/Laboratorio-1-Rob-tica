# Laboratorio-1-Robotica
Laboratorio 1 Robótica

Integrantes:

1. Luciano Alonso Cubillos Bugueño

2. Vicente Jose Montiel Torres

3. Sebastián Maximiliano Jeria López

4. Javiera Paz Cabrera Cácerez

5. Nidia Antonella Bugueño Rodríguez


## Solución del laboratorio 1

## Parte 1: Identificación de componentes y configuración

- Conectar Arduino UNO con el driver de motores y programar el movimiento básico de los motores (adelante, atrás, giro) sin controlar la velocidad.  
**Código:**

```
// Pines para motor izquierdo
const int IN1 = 8; 
const int IN2 = 9;
const int ENA = 10;  

// Pines para motor derecho
const int IN3 = 6;
const int IN4 = 7;
const int ENB = 5; 

void setup() {
// Configuración de los pines como salidas
pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

digitalWrite(ENA, HIGH);  
digitalWrite(ENB, HIGH);  
}

void loop() {
adelante();
delay(2000); 
detener();   

atras();
delay(2000); 
detener();   

giroIzquierda();
delay(2000); 
detener();   

giroDerecha();
delay(2000); 
detener();   

while (true); 
}

// Función para mover los motores hacia adelante
void adelante() {
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

// Función para mover los motores hacia atrás
void atras() {
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH);
}

// Función para girar a la izquierda
void giroIzquierda() {
digitalWrite(IN1, LOW);
digitalWrite(IN2, HIGH);
digitalWrite(IN3, HIGH);
digitalWrite(IN4, LOW);
}

// Función para girar a la derecha
void giroDerecha() {
digitalWrite(IN1, HIGH);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, HIGH);
}

// Función para detener los motores
void detener() {
digitalWrite(IN1, LOW);
digitalWrite(IN2, LOW);
digitalWrite(IN3, LOW);
digitalWrite(IN4, LOW);
}

```
Video: https://drive.google.com/file/d/1-7thUElghyUQU8m_MMpq4xIjezMeh6Hi/view?usp=sharing

- Verificar el funcionamiento del sensor ultrasónico HC-SR04 midiendo distancias.
  
**Código:**

```
const int trigPin = 9; 
const int echoPin = 10; 

void setup() {
pinMode(trigPin, OUTPUT);
pinMode(echoPin, INPUT);
Serial.begin(9600); 
}

void loop() {
// Pulso ultrasónico
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);

long duration = pulseIn(echoPin, HIGH);
float distance = duration * 0.034 / 2;

Serial.print("Distancia: ");
Serial.print(distance);
Serial.println(" cm");

delay(500);
}

```
Vídeo: https://drive.google.com/file/d/1ICvsdAPjJWLk06fB0XmYEiR-A8GNEfyz/view

- Analizar los datos del IMU MPUC6050 para medir inclinación o giros del robot.
**Código**
```
#include <Wire.h>
#include <MPU9250_asukiaaa.h>

MPU9250_asukiaaa mpu;

float gyroBiasX = 0, accelBiasX = 0;
const int N = 500;

void setup() {
Serial.begin(115200);
Wire.begin();
mpu.setWire(&Wire);
mpu.beginAccel();
mpu.beginGyro();
delay(1000);

Serial.println("Calibrando...");

for (int i = 0; i < N; i++) {
mpu.accelUpdate();
mpu.gyroUpdate();
gyroBiasX += mpu.gyroX();
accelBiasX += mpu.accelX();
delay(5);
}

gyroBiasX /= N;
accelBiasX /= N;

Serial.print("Bias del giroscopio X: ");
Serial.println(gyroBiasX);
Serial.print("Bias del acelerómetro X: ");
Serial.println(accelBiasX);
}

void loop() {
mpu.accelUpdate();
mpu.gyroUpdate();

float gyroX_corr = mpu.gyroX() - gyroBiasX;
float accelX_corr = mpu.accelX() - accelBiasX;

Serial.print("Giro corregido X: ");
Serial.print(gyroX_corr);
Serial.print(" | Aceleración corregida X: ");
Serial.println(accelX_corr);

  delay(100);


``` 
Robot estable datos: Rango de valores de Pitch (Kalman): aproximadamente entre -1.38 y -0.56.
El robot se encuentra con una leve inclinación negativa, posiblemente por la superficie, pero dentro de un rango aceptable que el sistema considera como estabilidad.
Robot inclinado datos:
Rango de Pitch (Kalman): aproximadamente entre -13.59 y -13.64.

El robot está inclinado significativamente (más de -13°), lo cual activa una condición de seguridad donde el sistema reduce la velocidad para evitar vuelcos o pérdida de control.

**Vídeo**: https://drive.google.com/file/d/1iSbEzJE6Q4Q0dANLKPghf5y5tCmUJBqs/view


# Preguntas 
**¿Qué función cumplen los sensores, actuadores y controladores en el robot?**

Respuesta: Los sensores son los órganos de los sentidos del robot, su función es entender el entorno, tomar decisiones y poder actuar autónomamente.
Los actuadores son dispositivos que convierten la energía en movimiento o fuerza para ejecutar acciones físicas.
Los controladores se encargan de recibir la información de los sensores y procesarla para tomar decisiones.

**¿Cómo se puede estimar la velocidad sin encoders?**

Respuesta: Estimando el desplazamiento a lo largo del tiempo (distancia/tiempo) usando sensores como el ultrasónico o midiendo el cambio de posición angular del IMU.

**¿Cómo afecta la falta de encoders a la precisión del movimiento?** 

Respuesta: Sin encoders el robot pierde precisión, esto hace que no haya un feedback sobre la posición del actuador, lo que hace difícil poder controlar los movimientos. Además, el robot puede ir más allá o no llegar a las distancias que tiene que recorrer.

**¿Qué es PWM y cómo ayuda a controlar la velocidad de los motores?**

Respuesta: PWM es una técnica para controlar la potencia suministrada a dispositivos. Ayuda a controlar la velocidad según cuánta energía le entrega al robot, un ciclo de trabajo más largo resulta en una mayor potencia y velocidad del motor, mientras que un ciclo más corto reduce la velocidad.

**¿Cómo afecta el control de velocidad a la precisión de la navegación sin encoders?** 

Respuesta: Si el motor no es controlado de forma precisa, puede variar la velocidad en función de la carga, fricción o desgaste del motor, lo que afecta a la dirección del robot.

## Parte 2: Cinemática y Dinámica de Robots Móviles usando un IMU 
- Aplicar la ecuación de cinemática diferencial para estimar la posición del robot usando tiempo y velocidad de motores.
**Código:**
```
#include <Wire.h>
#include <MPU6050.h>
MPU6050 imu;

// Pines para motor izquierdo
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10;
// Pines para motor derecho
const int IN3 = 6;
const int IN4 = 7;
const int ENB = 5;  

const int pwmValue = 100;
const float velocidad_estim = 0.15; // m/s

float x = 0.0, y = 0.0, theta = 0.0;
unsigned long prevTime = 0;
unsigned long startTime = 0;
bool motoresEncendidos = true;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  imu.initialize();

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  // Activar motores hacia adelante
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwmValue);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwmValue);

  startTime = millis();
  prevTime = startTime;
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - prevTime) / 1000.0;
  prevTime = currentTime;

  if (motoresEncendidos && (currentTime - startTime) >= 3000) {
    // Apagar motores después de 3 segundos
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    motoresEncendidos = false;
    Serial.println("Motores apagados después de 3 segundos.");
  }

  // Leer orientación
// Obtener velocidad angular (giroscopio Z)
int16_t gyroZ_raw = imu.getRotationZ();
float w = (gyroZ_raw / 131.0) * DEG_TO_RAD; // rad/s


  theta += w * dt;
  x += velocidad_estim * cos(theta) * dt;
  y += velocidad_estim * sin(theta) * dt;

  Serial.print("x: "); Serial.print(x, 2);
  Serial.print(" m | y: "); Serial.print(y, 2);
  Serial.print(" m | θ: "); Serial.print(theta * 180 / PI, 2); Serial.println("°");

  delay(100);
}


```
**Vídeo** : https://drive.google.com/file/d/1b2ujf7nt7txO1WAqVt-UNlxOgFwxKQ0A/view
- Hacer que el robot se mueva en línea recta y registrar desviaciones usando el sensor IMU para detectar la inclinación y giro del robot.
  
**Código:**

```
#include <Wire.h>
#include <MPU6050.h>
MPU6050 imu;

// Pines para motor izquierdo
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10; 

// Pines para motor derecho
const int IN3 = 6;
const int IN4 = 7;
const int ENB = 5;  

// PWM individual ajustado
const int pwmIzquierdo = 78;
const int pwmDerecho  = 100;

// Tiempo de movimiento en milisegundos
const unsigned long tiempoMovimiento = 3000;

unsigned long tiempoInicio = 0;
float theta = 0.0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  imu.initialize();

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  // Activar motores
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwmIzquierdo);

  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwmDerecho);

  tiempoInicio = millis();
}

void loop() {
  unsigned long ahora = millis();
  if (ahora - tiempoInicio >= tiempoMovimiento) {
    // Detener motores
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    Serial.println("Movimiento finalizado.");
    while (true); // Detener ejecución
  }

  // Leer velocidad angular en Z
  int16_t gyroZ_raw = imu.getRotationZ();
  float w = (gyroZ_raw / 131.0) * DEG_TO_RAD; // rad/s

  static unsigned long t_anterior = millis();
  float dt = (ahora - t_anterior) / 1000.0;
  t_anterior = ahora;

  theta += w * dt;
  float theta_deg = theta * 180 / PI;

  if (abs(theta_deg) > 2.0) {
    Serial.print("¡Desviación detectada! Ángulo: ");
    Serial.print(theta_deg, 2);
    Serial.println("°");
  }

  delay(50); // Lectura cada 50 ms aprox.
}



```
**Vídeo**: https://drive.google.com/file/d/1fpS-khl6iQy4GWuYIDFccKOfXM7q7GZq/view

-Usar el sensor IMU MPU6050 para medir la inclinación del robot y ajustar su dirección en tiempo real, realizando correcciones en el movimiento de acuerdo a su orientación.

**Código:**

```
#include <Wire.h>
#include <MPU6050.h>
MPU6050 imu;

// Pines para motor izquierdo
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10; 
// Pines para motor derecho
const int IN3 = 6;
const int IN4 = 7;
const int ENB = 5;

const int pwmBaseIzq = 78;
const int pwmBaseDer = 100;

const float Kp = 1.5;  // Ganancia proporcional 

float theta = 0.0;
unsigned long tiempoInicio = 0;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  imu.initialize();

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW); 
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);

  tiempoInicio = millis();
}

void loop() {
  unsigned long ahora = millis();
  if (ahora - tiempoInicio >= 4000) {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    Serial.println("Movimiento terminado.");
    while (true);
  }

  // Leer giroscopio Z
  int16_t gyroZ = imu.getRotationZ();
  float w = (gyroZ / 131.0) * DEG_TO_RAD;

  static unsigned long t_anterior = millis();
  float dt = (ahora - t_anterior) / 1000.0;
  t_anterior = ahora;

  theta += w * dt;
  float theta_deg = theta * 180 / PI;

  // Control proporcional
  float error = -theta_deg;
  float correccion = Kp * error;

  int pwmIzq = constrain(pwmBaseIzq + correccion, 0, 255);
  int pwmDer = constrain(pwmBaseDer - correccion, 0, 255);

  analogWrite(ENA, pwmIzq);
  analogWrite(ENB, pwmDer);

  Serial.print("θ: "); Serial.print(theta_deg, 2);
  Serial.print("° | PWM izq: "); Serial.print(pwmIzq);
  Serial.print(" | PWM der: "); Serial.println(pwmDer);

  delay(50);  
}


```
**Vídeo**: https://drive.google.com/file/d/1Zc-Ey_F4SHFxVSHrOlMefIYihA8CeaAN/view
- Programar el PWM para controlar la velocidad de los motores y hacer que el robot se mueva a diferentes velocidades sin IMU, variando el tiempo de activación de los motores.

**Código:**

```
// Pines para motor izquierdo
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10; 

// Pines para motor derecho
const int IN3 = 6;
const int IN4 = 7;
const int ENB = 5;  

const int velocidadesIzq[] = {78, 98, 118};  
const int velocidadesDer[] = {100, 120, 140};  

// Duración de cada velocidad (en milisegundos)
const int duraciones[] = {1500, 1500, 1500};

void setup() {
  Serial.begin(9600);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);

  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);

  // Ejecutar cada velocidad en secuencia
  for (int i = 0; i < 3; i++) {
    int pwmI = velocidadesIzq[i];
    int pwmD = velocidadesDer[i];

    Serial.print("Velocidad PWM Izquierdo: "); Serial.println(pwmI);
    Serial.print("Velocidad PWM Derecho : "); Serial.println(pwmD);

    analogWrite(ENA, pwmI);
    analogWrite(ENB, pwmD);

    delay(duraciones[i]);

    // Detener motores antes del siguiente ciclo
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    delay(1000); 
  }

  Serial.println("Secuencia completa.");
  while (true); // Detener ejecución
}

void loop() {
}



```
**Vídeo**: https://drive.google.com/file/d/1VQGqdG7C1GIXRpaNG7yKINdKvJK1Et1T/view

# Preguntas:
**¿Cómo se calcula la velocidad del robot sin encoders usando PWM?**
Respuesta: Sin encoders, no se puede medir directamente la velocidad real del robot. Sin embargo, se puede estimar la velocidad en función del valor de PWM aplicado al motor. Dado que el PWM controla el voltaje promedio que recibe el motor, se asume que a mayor PWM, mayor velocidad. La relación entre PWM y velocidad no es lineal debido a factores como fricción, carga y características del motor. Por lo tanto, se necesita una calibración previa experimental para correlacionar valores de PWM con velocidades aproximadas.

**¿Qué factores afectan la trayectoria y velocidad del robot al cambiar los intervalos de tiempo?**
Respuesta:
Intervalo de muestreo: Si el tiempo entre lecturas o ajustes de control es muy largo, el robot puede desviarse antes de que se corrija su rumbo.

Resolución temporal: Intervalos más cortos permiten respuestas más rápidas y precisas, pero también aumentan la carga computacional.

Acumulación de error: Con intervalos largos, los errores de dirección o velocidad no se corrigen a tiempo, afectando la trayectoria.

Ruido e inercia: El robot puede reaccionar de forma más inestable si los intervalos son demasiado cortos y no hay suficiente filtrado.

**¿Cuáles son las ventajas y desventajas de usar un IMU para ajustar la dirección en lugar de encoders?**
Respuesta:

**Ventajas:**

Mide orientación y aceleración, útil para detectar giros o inclinaciones.
No depende del contacto con las ruedas, por lo que funciona aunque haya derrapes.
Permite detectar cambios en la dirección sin necesidad de estar en movimiento.

**Desventajas:**

Acumula error con el tiempo (deriva), especialmente si se basa solo en giroscopio.

Requiere filtrado y calibración (por ejemplo, con un filtro complementario o de Kalman).

No mide distancia recorrida, por lo que no reemplaza totalmente a los encoders en estimación de posición.

**¿Qué efecto tiene la inclinación o el giro en el movimiento del robot, y cómo se corrige con el IMU?** 

Respuesta:

La inclinación o giro puede hacer que el robot cambie su rumbo o pierda estabilidad. Por ejemplo, una pendiente puede hacer que el robot acelere o desacelere sin cambiar el PWM, y un giro no deseado puede desviarlo de su trayectoria. El IMU detecta estos cambios midiendo aceleraciones y rotaciones. Usando esa información, el sistema de control puede ajustar la velocidad diferencial de los motores para corregir la dirección (por ejemplo, aplicando más PWM a un lado que al otro para girar y volver al rumbo deseado).










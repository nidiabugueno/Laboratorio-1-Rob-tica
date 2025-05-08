# Laboratorio-1-Rob-tica
Laboratorio 1 Robótica

Integrantes:
1.Luciano Alonso Cubillos Bugueño
2.Vicente Jose Montiel Torres
3.Sebastián Maximiliano Jeria López
4.Javiera Paz Cabrera Cácerez
5.Nidia Antonella Bugueño Rodríguez

## Solución del laboratorio 1

## Parte 1: Identificación de componentes y configuración

- Conectar Arduino UNO con el driver de motores y programar el movimiento básico de los motores (adelante, atrás, giro) sin controlar la velocidad.  
**Código:**

```
// En Arduino sería algo así:
void setup() {
  pinMode(4, OUTPUT); // IN1
  pinMode(5, OUTPUT); // IN2
  pinMode(6, OUTPUT); // ENA
}

void loop() {
  digitalWrite(4, HIGH);
  digitalWrite(5, LOW);
  analogWrite(6, 200); // Velocidad (0-255)
  delay(2000);

  digitalWrite(4, LOW);
  digitalWrite(5, HIGH);
  analogWrite(6, 200);
  delay(2000);
}
## Resultado del montaje
```
Video: https://drive.google.com/file/d/1eXlG0ELMHX4Q_qXAMt2ynprfgQdYbEFg/view?usp=sharing
- Verificar el funcionamiento del sensor ultrasónico HC-SR04 midiendo distancias.
**Código:**

```
#include <HCSR04.h>
#define PIN 13
#define TRIG 7
#define ECHO 8




void setup() {
  Serial.begin(9600);
  HCSR04.begin(TRIG, ECHO);


}


void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(PIN,HIGH);
  //delay(500);
  //digitalWrite(PIN,LOW);
  //delay(500);


  double* distances = HCSR04.measureDistanceCm();
 
  Serial.print("1: ");
  Serial.print(distances[0]);
  Serial.println(" cm");
 
  Serial.println("---");
  delay(250);


}
```
Vídeo: https://drive.google.com/file/d/1igSpsSvBTfWr4o2_Idc6r5wabalXJXAo/view
- Analizar los datos del IMU MPUC6050 para medir inclinación o giros del robot.
Robot estable datos: Rango de valores de Pitch (Kalman): aproximadamente entre -1.38 y -0.56.
El robot se encuentra con una leve inclinación negativa, posiblemente por la superficie, pero dentro de un rango aceptable que el sistema considera como estabilidad.
Robot inclinado datos:
Rango de Pitch (Kalman): aproximadamente entre -13.59 y -13.64.
El robot está inclinado significativamente (más de -13°), lo cual activa una condición de seguridad donde el sistema reduce la velocidad para evitar vuelcos o pérdida de control.
Vídeo: https://drive.google.com/file/d/1NLxmeG9EuRUX8IRj0OA2ZexnuW2RMgMh/view

# Preguntas 
¿Qué función cumplen los sensores, actuadores y controladores en el robot? 
Respuesta: Los sensores son los órganos de los sentidos del robot, su función es entender el entorno, tomar decisiones y poder actuar autónomamente.
Los actuadores son dispositivos que convierten la energía en movimiento o fuerza para ejecutar acciones físicas.
Los controladores se encargan de recibir la información de los sensores y procesarla para tomar decisiones.

¿Cómo se puede estimar la velocidad sin encoders? 
Respuesta: Estimando el desplazamiento a lo largo del tiempo (distancia/tiempo) usando sensores como el ultrasónico o midiendo el cambio de posición angular del IMU.

¿Cómo afecta la falta de encoders a la precisión del movimiento? 
Respuesta: Sin encoders el robot pierde precisión, esto hace que no haya un feedback sobre la posición del actuador, lo que hace difícil poder controlar los movimientos. Además, el robot puede ir más allá o no llegar a las distancias que tiene que recorrer.

¿Qué es PWM y cómo ayuda a controlar la velocidad de los motores?
Respuesta: PWM es una técnica para controlar la potencia suministrada a dispositivos. Ayuda a controlar la velocidad según cuánta energía le entrega al robot, un ciclo de trabajo más largo resulta en una mayor potencia y velocidad del motor, mientras que un ciclo más corto reduce la velocidad.

¿Cómo afecta el control de velocidad a la precisión de la navegación sin encoders? 
Respuesta: Si el motor no es controlado de forma precisa, puede variar la velocidad en función de la carga, fricción o desgaste del motor, lo que afecta a la dirección del robot.

## Parte 2: Cinemática y Dinámica de Robots Móviles usando un IMU 







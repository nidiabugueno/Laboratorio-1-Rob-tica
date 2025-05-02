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

```cpp
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


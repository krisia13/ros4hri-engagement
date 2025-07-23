# **Protocolo de Evaluación de Usuario**

## **1. Introducción**

- Presenta brevemente el proyecto y el objetivo general del robot (por ejemplo: “Este robot está diseñado para navegar de forma autónoma y responder a la interacción humana mediante reconocimiento de voz y detección de presencia”).
- Explica que la sesión constará de dos partes: una prueba libre y una prueba guiada.
- Informa al usuario de que puede preguntar cualquier duda y que sus respuestas y comportamientos serán anónimos y solo usados para mejorar el sistema.

---

## **2. Prueba Libre**

### **Objetivo**
Observar el comportamiento espontáneo del usuario, sus expectativas y la naturalidad de la interacción sin instrucciones previas.

### **Desarrollo**

1. **Situación inicial:**  
   El robot está navegando de forma autónoma entre waypoints.

2. **Instrucción al usuario:**  
   “Mientras el robot navega, te voy a contar en qué consiste el proyecto. No te voy a dar instrucciones específicas sobre cómo interactuar con el robot. Si te apetece, puedes acercarte, hablarle o simplemente observarlo. Haz lo que te salga de forma natural.”

3. **Observaciones a registrar:**  
   - ¿El usuario intenta interactuar con el robot? ¿Cómo?
   - ¿Se acerca, le habla, le toca, le bloquea el paso, etc.?
   - ¿Qué espera que haga el robot?
   - ¿El usuario muestra dudas, miedo, curiosidad?
   - ¿El robot responde de alguna manera a estos comportamientos espontáneos?

4. **Preguntas tras la prueba libre:**  
   - ¿Qué pensabas que podías hacer con el robot?
   - ¿Intentaste interactuar de alguna manera? ¿Cómo?
   - ¿El robot respondió como esperabas?
   - ¿Qué te habría gustado que hiciera?
   - ¿Te ha resultado fácil o difícil entender cómo interactuar con él?
   - ¿Qué mejorarías en la interacción?

---

## **3. Prueba Guiada**

### **Objetivo**
Evaluar funcionalidades específicas del sistema y la experiencia de usuario en situaciones controladas.

### **Desarrollo**

#### **Caso 1: Navegación autónoma y conversación de fondo**
- **Instrucción:**  
  “Observa cómo el robot navega solo entre los puntos. Mientras tanto, hablaremos normalmente cerca de él, pero sin intentar interactuar directamente.”
- **Objetivo:**  
  Comprobar que el robot ignora conversaciones de fondo y solo reacciona a interacciones dirigidas.

#### **Caso 2: Detección de engagement**
- **Instrucción:**  
  “Acércate al robot y ponte delante de él, como si quisieras interactuar. Observa si el robot detecta tu presencia y cambia su comportamiento.”
- **Objetivo:**  
  Evaluar la detección de intención de interacción (engagement).

#### **Caso 3: Interacción por voz con palabra clave**
- **Instrucción:**  
  “Mientras estás cerca del robot, háblale usando frases normales, pero incluye la palabra ‘hola’ en alguna de ellas, por ejemplo: ‘Hola, ¿puedes mirarme?’ '¿Qué tal?, hola, ¿puedes ayudarme?'”
- **Objetivo:**  
  Evaluar el reconocimiento de la palabra clave y la reacción del robot (giro, respuesta).

#### **Caso 4: Giro hacia la voz**
- **Instrucción:**  
  “Cuando el robot te mire y diga que te está mirando, espera unos segundos y observa si vuelve a navegar.”
- **Objetivo:**  
  Comprobar que el robot completa la interacción y retoma su tarea.

#### **Caso 5: Repetición desde otra posición**
- **Instrucción:**  
  “Cuando el robot vuelva a navegar, repite el saludo ‘hola’ desde otra dirección y observa si gira correctamente hacia ti.”
- **Objetivo:**  
  Evaluar la robustez del sistema ante múltiples interacciones.

#### **Caso 6: Interacción durante el giro**
- **Instrucción:**  
  “Mientras el robot está girando hacia ti, intenta decir ‘hola’ de nuevo y observa si reacciona o ignora el comando.”
- **Objetivo:**  
  Comprobar que el robot no procesa comandos de voz mientras está ocupado.

#### **Caso 7: Engagement tras el giro y saludo**
- **Instrucción:**  
  “Cuando el robot gire hacia ti y diga ‘I am looking at you now. Did you want something?’, acércate un poco más y ponte justo delante de él, mostrando claramente tu intención de interactuar (por ejemplo, mirándole o saludando con la mano). Observa si el robot detecta tu presencia y entra en modo engagement (por ejemplo, se detiene, cambia luces, etc.).”
- **Objetivo:**  
  Evaluar si el robot detecta correctamente el engagement justo después de girar y saludar, y si responde adecuadamente a la intención clara de interacción del usuario.  

#### **Caso 8: Fin de la interacción**
- **Instrucción:**  
  “Aléjate del robot y observa si vuelve a su navegación normal.”
- **Objetivo:**  
  Comprobar la recuperación del modo autónomo tras la interacción.

---

### **Preguntas tras la prueba guiada**
- ¿Te ha parecido natural la reacción del robot en cada caso?
- ¿Te ha resultado fácil provocar la reacción del robot?
- ¿Notas algún fallo o algo que mejorarías?
- ¿Te gustaría que el robot respondiera a otras palabras o comportamientos?
- ¿Te has sentido cómodo/a interactuando con el robot?

---

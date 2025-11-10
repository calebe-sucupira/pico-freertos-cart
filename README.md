# Carrinho Desvia-Obstáculo com FreeRTOS na Pi Pico

Este projeto é um robô móvel autônomo construído sobre a plataforma **Raspberry Pi Pico (ARM Cortex-M0+)** que utiliza o Sistema Operacional de Tempo Real **FreeRTOS** para navegação e tomada de decisão.

O robô navega para frente até detectar um obstáculo. Ao detectá-lo, ele para, escaneia o ambiente em 180 graus usando um servo e um sensor ultrassônico, e então decide o melhor caminho (esquerda ou direita) para continuar.


---

## 🛠️ Hardware Utilizado

* **Microcontrolador:** Raspberry Pi Pico
* **Sensor:** Sensor Ultrassônico HC-SR04 (para medição de distância)
* **Atuador (Scan):** Servo Motor SG90 (para girar o sensor)
* **Atuador (Movimento):** 2x Motores DC (controlados diretamente pelos pinos GPIO)

## ⚙️ Arquitetura de Software (FreeRTOS)

A lógica do sistema é dividida em **quatro tarefas concorrentes** que se comunicam usando mecanismos de IPC (Comunicação Inter-Processos) do FreeRTOS, garantindo uma operação reativa e sem bloqueios.

### 1. `vTaskMeasureDistance` (Prioridade 3 - Alta)

* **Função:** A tarefa mais crítica. É a única responsável por interagir com o sensor ultrassônico (HC-SR04).
* **Lógica:** Executa a cada 25ms, envia o pulso de `TRIG` e mede o pulso de `ECHO`.
* **IPC:** Armazena a distância medida (`last_distance`) em uma variável global protegida por um **Mutex** (`xDistanceMutex`) para ser consumida por outras tarefas com segurança.
    

### 2. `vTaskMove` (Prioridade 2 - Média)

* **Função:** A tarefa de "ação". Responsável por mover o carrinho para frente.
* **Lógica:** Liga os motores e verifica continuamente (via `check_for_obstacle()`) se um obstáculo foi detectado (lendo `last_distance` e setando `obstacle_detected`).
* **IPC:**
    * Ao detectar um obstáculo, para os motores e envia uma **Notificação de Tarefa** (`xTaskNotifyGive`) para a `vTaskScan`.
    * Em seguida, se auto-suspende (`vTaskSuspend(NULL)`) para aguardar uma decisão.
    

### 3. `vTaskScan` (Prioridade 1 - Baixa)

* **Função:** A tarefa de "percepção". Mapeia o ambiente.
* **Lógica:** Aguarda uma notificação da `vTaskMove` (`ulTaskNotifyTake`). Ao receber, gira o servo de 0 a 180 graus.
* **IPC:** A cada grau, lê a distância atual (via `last_distance`) e envia o valor `float` para a `distanceQueue`.
    * Ao completar os 180 graus, envia uma **Notificação de Tarefa** para a `vTaskDecision`.
  

### 4. `vTaskDecision` (Prioridade 1 - Baixa)

* **Função:** A tarefa de "inteligência". Decide o próximo movimento.
* **Lógica:** Aguarda uma notificação da `vTaskScan`. Ao receber, esvazia a `distanceQueue`, processa todas as 180 leituras de distância, e compara a média/mínima da esquerda com a da direita.
* **IPC:**
    * Toma a decisão (virar à esquerda ou à direita) e executa o movimento.
    * Ao final, "acorda" a tarefa `vTaskMove` (usando `vTaskResume(moveTaskHandle)`) para que o ciclo recomece.

---

### Código Fonte

* **`main.c`**: Contém toda a lógica do FreeRTOS, criação de tarefas, handlers de IPC e a máquina de estados principal.
* **`servo.c`**: Biblioteca (driver) para controle do servo motor via PWM de hardware do Pico.
# Robocode-com-IA

🤖 SuperSpinBot - Robocode com INTELIGÊNCIA ARTIFICIAL

Projeto desenvolvido para a disciplina de INTELIGÊNCIA ARTIFICIAL do curso de CIÊNCIA DA COMPUTAÇÃO.

O objetivo deste trabalho foi desenvolver um robô para o ambiente **Robocode** capaz de adaptar sua estratégia durante as batalhas utilizando técnicas de Inteligência Artificial e aprendizado baseado em experiência.

---

## 📖 Sobre o projeto

O "SuperSpinBot" é um robô desenvolvido em Java para o simulador Robocode que utiliza múltiplas estratégias de mira e um mecanismo de aprendizado para selecionar automaticamente a melhor estratégia de acordo com o comportamento do inimigo.

Durante a batalha, o robô coleta informações dos disparos realizados, aprende com os resultados obtidos e utiliza essas informações para melhorar sua tomada de decisão ao longo do combate.

---

🛠 Tecnologias utilizadas

- Java
- Robocode
- Programação Orientada a Objetos (POO)
- Inteligência Artificial
- Algoritmo K-Nearest Neighbors (KNN)
- Estatística adaptativa
- Predição de movimento

---

🧠 Estratégias de Inteligência Artificial

O robô utiliza diferentes abordagens de IA para aumentar sua precisão.

1. Aprendizado por desempenho

Cada estratégia de mira possui estatísticas próprias de:

- quantidade de disparos;
- quantidade de acertos;
- taxa de sucesso.

Essas informações são utilizadas para calcular qual estratégia apresenta melhor desempenho durante a batalha.

---

2. K-Nearest Neighbors (KNN)

O principal algoritmo de IA utilizado foi o KNN (K-Nearest Neighbors).

Cada disparo gera uma amostra contendo:

- distância do inimigo;
- velocidade lateral;
- velocidade do inimigo;
- estratégia utilizada;
- resultado (acerto ou erro).

Quando há uma quantidade suficiente de dados armazenados, o robô procura os casos mais semelhantes ao cenário atual e estima qual estratégia possui maior probabilidade de acerto.

Essa abordagem permite que o robô adapte sua mira de acordo com o comportamento observado do adversário.

---

3. Aprendizado da velocidade do inimigo

Além do KNN, o robô mantém um histórico das velocidades do inimigo.

Essas informações são utilizadas para calcular uma velocidade média esperada em diferentes situações, permitindo melhorar a predição de movimento durante disparos com mira circular.

---

🎯 Estratégias de mira

O robô possui três modos diferentes de ataque.

## Head-On Targeting

Dispara diretamente na posição atual do inimigo.

É uma estratégia simples, porém eficiente contra robôs com pouca movimentação.

---

### Linear Targeting

Prediz a posição futura do inimigo assumindo que ele continuará se movimentando em linha reta.

Essa estratégia melhora significativamente a precisão contra movimentos constantes.

---

### Circular Targeting

Prediz a trajetória considerando mudanças de direção do inimigo.

Nesta implementação, a previsão ainda utiliza o aprendizado da velocidade média observado durante a batalha.

---

🚀 Estratégia de movimentação

O movimento do robô é baseado em uma estratégia circular com mudanças aleatórias.

Características:

- movimento contínuo;
- alteração dinâmica da velocidade;
- mudança de direção quando detecta disparos do inimigo;
- tentativa de dificultar a previsão por parte dos adversários.

Esse comportamento reduz a chance de ser atingido.

---

📊 Processo de aprendizagem

Durante toda a batalha o robô:

1. dispara utilizando uma estratégia escolhida;
2. registra os dados daquele disparo;
3. verifica posteriormente se houve acerto;
4. atualiza sua base de conhecimento;
5. utiliza essas informações para escolher melhores estratégias nos próximos disparos.

Assim, o robô melhora sua tomada de decisão conforme acumula experiência.

---

🎓 Objetivos acadêmicos

Este trabalho teve como objetivo aplicar conceitos estudados na disciplina de Inteligência Artificial, incluindo:

- aprendizado baseado em experiência;
- classificação utilizando KNN;
- tomada de decisão adaptativa;
- sistemas inteligentes;
- predição de comportamento;
- análise estatística de desempenho.

---

▶️ Como executar

1. Instale o Robocode.
2. Adicione a classe `SuperSpinBot.java` ao diretório de robôs.
3. Compile o projeto.
4. Execute uma batalha no Robocode.
5. Observe o comportamento adaptativo do robô durante o combate.

---

## 👨‍💻 Autor

Projeto desenvolvido como trabalho da disciplina de INTELIGÊNCIA ARTIFICIAL do curso de CIÊNCIA DA COMPUTAÇÃO.

---

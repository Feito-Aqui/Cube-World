# eRemake Cube World

Este projeto tem como objetivo recriar o clássico **Cube World** utilizando a placa de desenvolvimento **ESP32-S3 1.69inch Touch Display Development Board**. Em vez de usar os tradicionais **pogo pins** para conectar os cubos como no original, vamos usar **Bluetooth (BT)** para a comunicação entre eles. A ideia é implementar as icônicas animações e interações entre os cubos do jogo utilizando a biblioteca **LVGL** (LittlevGL).

## Tecnologias Utilizadas

- **ESP32-S3 1.69inch Touch Display Development Board**: Placa de desenvolvimento que conta com tela sensível ao toque de 1,69 polegadas e o poderoso ESP32-S3, ideal para projetos gráficos e Bluetooth.
- **LVGL (LittlevGL)**: Biblioteca gráfica leve para microcontroladores, que será utilizada para recriar as animações e interações no estilo Cube World.
- **Bluetooth (BT)**: Protocolo de comunicação sem fio usado para conectar os cubos, substituindo os antigos pogo pins.
- **ESP-IDF**: Framework de desenvolvimento para ESP32, utilizado para gerenciar o hardware e a comunicação entre os dispositivos.

## Objetivo do Projeto

O principal objetivo deste projeto é recriar a experiência do Cube World de maneira moderna, com:

- **Animações de Cubo**: Recriação das animações icônicas do Cube World usando LVGL para interações visuais.
- **Conexão sem fio**: Substituição dos pogo pins por uma conexão Bluetooth entre os cubos.
- **Touchscreen**: Interação com os cubos e animações diretamente na tela sensível ao toque do ESP32-S3.

## Funcionalidades

- **Comunicação entre Cubos**: Os cubos se conectam e se comunicam via Bluetooth, trocando informações e status.
- **Interações e Animações**: Animações fluidas baseadas nos movimentos e interações dos cubos, com o controle de interações sensíveis ao toque.
- **Interface Gráfica**: Utilização de LVGL para criar uma interface interativa, com elementos gráficos semelhantes aos do Cube World original.

# simulador

https://gitlab.com/eufs/eufs_sim/-/wikis/Getting-Started-Guide

Seguindo os passos do original e possivel compilar normalmente, para evitar problemas, compile o ros-galactic tambem, nao baixe pelo APT, pois podem faltar pacotes.

Problemas compilando rqt ou rqt_graph? 
```
# Solução: https://answers.ros.org/question/398497/building-from-source-rqt-fails/ 
pip install setuptools==58.2.0
```
Caso dê problema rodando ros2 launch, faça `sudo apt install ros-galactic-desktop`

Para suporte ao controle de PS4 faça `pip install pyPS4Controller`

TODO:
- [ ] adicionar o logo da universidade como textura do ground plane

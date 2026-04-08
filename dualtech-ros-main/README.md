# DualTech ROS

Repozytorium zawiera przykładowy kod do wykorzystania w ramach zawodów Dual Tech AGH.
Stanowi ono przy tym małą przestrzeń roboczą pakietu ROS, wzbogaconą dodatkowo o kontener Docker.
W repozytorium znajdują się dwie paczki ROS:
1. [dualtech](./dualtech/) - zawiera ona przykłady *publishera* oraz *subscribera*, które komunikują się ze sobą za pomocą wiadomości zbliżonej do tej, która będzie wymagana w trakcie zawodów.
2. [dualtech_msgs](./dualtech_msgs/) - definicja w/w wiadomości.

W docelowym systemie, wspomniany *publisher* powinien być uruchomiony bezpośrednio na Raspberry Pi i wysyłać wyniki detekcji, natomiast *subscriber* będzie działał po stronie Organizatorów, aby w ten sposób na bieżąco weryfikować rezultaty zadania.
W procesie testowania własnego rozwiązania można wykorzystać kontener Docker, aby uruchomić *subscriber* na własnym komputerze i w ten sposób symulować warunki panujące w trakcie zawodów.

## Pakiet ROS

ROS (Robot Operating System) stanowi framework wykorzystywany intensywnie do opracowywania aplikacji robotycznych, szczególnie w obszarze robotyki mobilnej i systemów rozproszonych.
Umożliwia on integrację pracy różnych komponentów reprezentowanych przez węzły (*node*), które mogą wymieniać informacje za pośrednictwem tematów (*topic*), serwisów (*service*) lub akcji (*action*).

Z perspektywy systemu wykorzystywanego w ramach konkursu, istotna jest jeydnie komunikacja za pośrednictwem tematów.
W repozytorium znajduje się przykład węzła *detection_publisher*, który publikuje zbiorczą informację o wyniku detekcji w postaci (ID obiektu, typ-nazwa obiektu, obraz).
Wystarczy jedynie zintegrować ten fragment z własnym kodem i podstawić wynik w odpowiednie miejsca.

W celu lepszego zrozumienia działania pakietu ROS warto przerobić [oficjalne tutoriale](https://docs.ros.org/en/jazzy/Tutorials.html), w szczególności te przeznaczone dla początkujących (dwie części "Beginner").

### ROS na Raspberry Pi

W celu wykorzystania ROS-a na Raspberry Pi należy go najpierw tam zainstalować, zgodnie z przygotowanym [tutorialem](./install_rpi_ros2.md).
Następnie tworzymy nasz katalog roboczy, np. w folderze home:
``` bash
mkdir -p ros2_ws/src

```

W folderze `src` należy umieścić zawartość repozytorium, a następnie zbudować z poziomu katalogu roboczego
``` bash
cd ~/ros2_ws
colcon build --symlink-install --merge-install
```

W efekcie katalog roboczy "wzbogaci" się o nowe foldery: `build`, `install`, `log`.
Aby umożliwić dostęp do zbudowanych programów z poziomu terminala należy jeszcze dodać potrzebne ścieżki za pomocą automatycznie generowanego skryptu:
``` bash
source ~/ros2_ws/install/setup.bash
```

Teraz można sprawdzić, czy wszystko działa poprawnie uruchamiając węzeł *detection publisher*:
``` bash
ros2 run dualtech detection_publisher
```

## Kontener Docker

W celu wykorzystania kontenera należy w pierwszej kolejności zainstalować oprogramowanie [Docker](https://docs.docker.com/get-started/) na swoim komputerze.
Następnie zaleca się uruchomienie kontenera poprzez [VS Code](https://code.visualstudio.com/), przy pomocy dedykowanego rozszerzenia [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) wspierającego obsługę Dockera.
Wówczas wystarczy jedynie uruchomić `Command Palette` programu VS Code i wyszukać polecenie `Dev Containers: Reopen in Container`.
Alternatywnie można również wykorzystać natywne wsparcie Dockera, zgodnie z [dokumentacją](https://docs.docker.com/reference/).

Przy pierwszym uruchomieniu kontenera należy najpierw wstępnie skonfigurować środowisko, a następnie zbudować cały folder roboczy za pomocą następujących komend:
``` bash
cd /home/developer/ros2_ws
./setup.sh
./build.sh
```

> [!TIP]
> W przypadku korzystania z systemu Windows zaleca się wykorzystanie WSL2 z systemem Ubuntu 24.04.
> Drzewo folderów z systemu Windows jest typowo montowane wewnątrz WSL2 jako katalog `/mnt`, który jednak zachowuje system plików NTFS i nieco inne uprawnienia odczytu/zapisu.
> Z tego względu najbezpieczniej umieścić zawartość repozytorium bezpośrednio w katalogu `$HOME` systemu Ubuntu wewnątrz WSL2
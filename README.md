![roveri](/images/roveri.svg)

# GetPose

ROS2-sovellus mobiilirobotin paikka- ja asentotiedon lukemiseen ja tallentamiseen.

## Sovelluksen toiminta

Sovellus lukee */amcl_pose* aihetta (engl. topic), johon erillinen ROS2-sovellus julkaisee lasketun arvion mobiilirobotin sijainnista. 

Alla on esimerkki viestistä, joka on luettu */amcl_pose*-aiheeseesta.
```
header:
stamp:
    sec: 1695155415
    nanosec: 1783364
frame_id: map
pose:
pose:
    position:
    x: -0.24965593884405274
    y: 0.0027770199910801233
    z: 0.0
    orientation:
    x: 0.0
    y: 0.0
    z: -0.0030858282177376063
    w: 0.9999952388207709
covariance:
- 0.0
- ...
```

Sovellus muuntaa ROS2-viestin JSON-formaattiin ja kirjoittaa sen *resource*-alikansiossa olevaan *amclpose.log*-tiedostoon. Sijaintitieto sisältää aikaleiman (*ts = timestamp*) muodossa *sekunnit.nanosekunnit*, sijainnin *p* = (*px*, *py*, *pz*), ja kvaternion *q* = (*qx*, *qy*, *qz*, *qw*) alla olevan esimerkin mukaisesti.
```
{
    "ts": 1695155415.1783364, 
    "px": -0.24965593884405274, 
    "py": 0.0027770199910801233, 
    "pz": 0.0, 
    "qt": [0.0, 0.0, -0.0030858282177376063, 0.9999952388207709]
}
```

Sijaintitieto julkaistaan noin kerran sekunnissa. Huomaa, että uutta sijaintitietoa ei kuitenkaan julkaista, jos mobiilirobotti on paikallaan.

## Sijainnin lukeminen terminaalissa

Mobiilirobotin sijaintitietoa voidaan kysyä myös ilman tätä sovellusta suoraan terminaalista:
```
ros2 topic echo /amcl_pose
```

Tarkemmat tiedot sijaintiedon viestistä saa komennolla:
```
ros2 topic info /amcl_pose --verbose
```

## Sovelluksen ajaminen

Käynnistä ensimmäisessä terminaalissa ROS2-sovellus, jolla mahdollistaa TurtleBotin ohjaamisen tietokoneen näppäimistöllä. Alla olevan linkin kautta näkee millä näppäimillä TurtleBottia ohjataan.

[https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html](https://turtlebot.github.io/turtlebot4-user-manual/tutorials/driving.html)

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Kopioi *resource* alikansiossa olevat *robolabra.pgm* ja *robolabra.yaml* tiedostot Ubuntun *Home* kansioon. Käynnistä sitten toisessa terminaalissa paikannus ja anna sille parametrina paikannuksessa käytettävän karttapohjan YAML-tiedosto.
```
ros2 launch turtlebot4_navigation localization.launch.py map:=robolabra.yaml
```

Käynnistä lopuksi tämä sovellus kolmannessa terminaalissa.
```
cd myROS2workspace
source install/local_setup.bash
ros2 run getpose main
```

## Tekijätiedot

Hannu Hakalahti, Asiantuntija TKI, Seinäjoen ammattikorkeakoulu

## Hanketiedot

* Hankkeen nimi: Autonomiset ajoneuvot esiselvityshanke
* Rahoittaja: Töysän säästöpankkisäätiön tutkimusrahasto
* Aikataulu: 01.08.2023 - 31.06.2024
---
![rahoittajan_logo](/images/toysan_sp_saatio.jpg)

![seamk_logo](/images/SEAMK.jpg)
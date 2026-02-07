# ros2_esp32_prr_manipulator

# ROS 2 & ESP32 Robot Arm Control Project

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-22314E?style=for-the-badge&logo=ros&logoColor=white)
![MoveIt](https://img.shields.io/badge/MoveIt-Motion_Planning-blue?style=for-the-badge)
![Micro-ROS](https://img.shields.io/badge/Micro--ROS-Hardware_Bridge-ff69b4?style=for-the-badge)
![ESP32](https://img.shields.io/badge/ESP32-Microcontroller-success?style=for-the-badge&logo=espressif)
![Docker](https://img.shields.io/badge/Docker-Container-2496ED?style=for-the-badge&logo=docker&logoColor=white)

## Proje Hakkında (About the Project)

Bu proje, **ROS 2 Jazzy** ve **MoveIt 2** kullanılarak simüle edilen 3 eksenli (3-DOF) bir robot kolun, **Micro-ROS** aracılığıyla gerçek zamanlı olarak fiziksel bir robotla (ESP32 ve Servo Motorlar) senkronize edilmesini sağlar.

Proje, modern robotik mimarisine uygun olarak **Dağıtık Sistem (Distributed System)** mantığıyla çalışır. PC üzerinde çalışan yapay zeka ve planlama algoritmaları (MoveIt), Docker üzerinde çalışan bir Micro-ROS Agent aracılığıyla gömülü sistemle (ESP32) haberleşir.

### Temel Özellikler
* **Hibrit Eklem Yapısı:** Hem Prizmatik hem de Revolute eklemlerin kontrolü.
* **Tam Senkronizasyon:** Gazebo simülasyonu ile gerçek robotun eş zamanlı hareketi.
* **Dockerize Edilmiş Agent:** Micro-ROS Agent, Docker konteyneri içinde izole çalışır.

---

## Sistem Mimarisi (System Architecture)

Sistem 3 ana katmandan oluşur:

1.  **Kontrol Katmanı (PC / ROS 2):**
    * MoveIt 2 (Yörünge Planlama)
    * Gazebo (Fizik Simülasyonu)
    * `/joint_states` topic yayını
2.  **Köprü Katmanı (Docker):**
    * Micro-ROS Agent (Serial Transport)
3.  **Donanım Katmanı (ESP32):**
    * Micro-ROS Node (Subscriber)
    * MG996R Servo Motor Sürücüleri

---

## Donanım Bağlantısı (Hardware Setup)

**Uyarı:** MG996R motorları yüksek akım çeker. ESP32'nin 5V pininden beslemeyiniz. Harici bir 5V-6V güç kaynağı kullanın ve **GND'leri birleştirmeyi unutmayın.**

| Robot Eklemi | Servo Tipi | ESP32 Pini (GPIO) |
| :--- | :--- | :--- |
| **Prismatic** | MG996R | `GPIO 13` |
| **Omuz (Shoulder)** | MG996R | `GPIO 12` |
| **Dirsek (Elbow)** | MG996R | `GPIO 14` |

---

## Kurulum ve Çalıştırma (Installation & Usage)

### 1. Gereksinimler
* Ubuntu 24.04 (Noble Numbat)
* ROS 2 Jazzy Jalisco
* Docker
* Arduino IDE (ESP32 Board Manager & Micro-ROS Library)

### 2. Depoyu Klonlayın
```bash
git clone https://github.com/csayar-TR/ros2_esp32_prr_manipulator.git

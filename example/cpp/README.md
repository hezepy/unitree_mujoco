# H1 Inspire Hand Service

Unitree H1 Inspire Hand Controller.

## Usage

On H1 PC2 terminal, in folder of h1_inpire: 

```bash
# Build project
mkdir build & cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
# Terminal 1. Run h1 inspire hand service
sudo ./inspire_hand -s /dev/ttyUSB0
```
On local PC terminal, in this folder: 

```bash
# Terminal 2. Run example
./h1_hand enp3s0 # enp3s0 is netport name
```

# H1 Damiao Wrist Motor Service

Unitree H1 Wrist Motor Controller.

## Usage

On H1 PC2 terminal, in folder of h1_damiao: 

```bash
mkdir build & cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make
# Terminal 1. Run h1 damiao service
sudo ./h1_damiao -s /dev/ttyACM0
```
On local PC terminal, in this folder: 

```bash
# Terminal 2. Run example
./h1_wrist enp3s0 # enp3s0 is netport name
```
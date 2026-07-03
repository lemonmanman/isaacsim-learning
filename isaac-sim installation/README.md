# Isaac Sim Installation Tips
Some tips (mostly personal attempts)
## isaac sim version
Isaac Sim Full 6.0.0

You can download it from [Isaacsim](https://github.com/isaac-sim/IsaacSim/tree/develop?tab=readme-ov-file)

> Note: Make sure whether you are git cloning the main branch or the develop branch, while the former results in Isaac Sim 5.1.0 and the ladder results in Isaac Sim 6.0.0.

You can use **git checkout** to switch to the **develop** branch in order to download the latest version.

## Using distrobox to install 5.0
First, create the distrobox:
```bash
mkdir -p ~/distrobox/ubuntu22

# use "--nvidia" to create
distrobox create --name ubuntu22 --image ubuntu:22.04 --home ~/distrobox/ubuntu22 --nvidia
```

After creating the distrobox, you need to install packages below:
```bash
sudo apt-get install lsb-release build-essential git git-lfs libglu1-mesa libxt6
```

For the following steps, you can refer to the [Quick Install](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/installation/quick-install.html).
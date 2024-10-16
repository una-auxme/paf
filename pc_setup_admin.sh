if [ $# -eq 0 ]; then
    echo "Usage: $0 <pafxxx-username>"
    exit 1
fi

sudo apt update
sudo apt upgrade -y
sudo apt install -y git gh python-is-python3 python3-pip openssh-server
sudo systemctl enable ssh

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo groupadd docker
sudo usermod -aG docker "$1"

# NVIDIA
# https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg \
  && curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update

sudo apt-get install -y nvidia-container-toolkit

sudo nvidia-ctk runtime configure --runtime=docker # configure the runtime to use nvidia-container-runtime

sudo systemctl restart docker

sudo nvidia-ctk runtime configure --runtime=docker --config=/home/"$1"/.config/docker/daemon.json
sudo chown -R "$1":"$1" /home/"$1"/.config

sudo systemctl restart docker

# Could be removed, because it does not appear to work
sudo nvidia-ctk config --set nvidia-container-cli.no-cgroups=false --in-place

sudo snap install --classic code

sudo nano /etc/nvidia-container-runtime/config.toml

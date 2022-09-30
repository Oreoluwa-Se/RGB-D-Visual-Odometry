mkdir -p ~/webpage_ws/nvm
export NVM_DIR="/home/user/webpage_ws/nvm"
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh | bash
ls ~/webpage/nvm`
source ~/.bashrc
nvm install v14
nvm alias default v14
npm install -g http-server
# PC Setup

- Install Ubuntu 24.04 LTS
- Login to Admin user
- Create new pafxxx user
- execute `pc_setup_admin.sh` as admin user with `pafxxx` as argument
- change user to `pafxxx`
- execute `pc_setup_user.sh` as new user with `pafxxx` as argument
- In ~/.bashrc, add
  
```bash
export PAF_USERNAME=$(id -u -n)
export PAF_UID=$(id -u)
export PAF_GID=$(id -g)
```

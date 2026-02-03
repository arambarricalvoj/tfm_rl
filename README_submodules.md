javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl/create3_sim_JavAram/create3_ws/src$ git clone git@github.com:robo-friends/m-explore-ros2.git
Cloning into 'm-explore-ros2'...
remote: Enumerating objects: 505, done.
remote: Counting objects: 100% (230/230), done.
remote: Compressing objects: 100% (107/107), done.
remote: Total 505 (delta 174), reused 126 (delta 123), pack-reused 275 (from 2)
Receiving objects: 100% (505/505), 209.01 KiB | 748.00 KiB/s, done.
Resolving deltas: 100% (267/267), done.
javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl/create3_sim_JavAram/create3_ws/src$ cd m-explore-ros2/
javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl/create3_sim_JavAram/create3_ws/src/m-explore-ros2$ ls -lias
total 48
51124141 4 drwxrwxr-x 7 javierac javierac 4096 Feb  3 10:23 .
51132229 4 drwxrwxr-x 6 javierac javierac 4096 Feb  3 10:23 ..
51124257 4 -rw-rw-r-- 1 javierac javierac 1458 Feb  3 10:23 .clang-format
51124304 4 drwxrwxr-x 2 javierac javierac 4096 Feb  3 10:23 .devcontainer
51124324 4 drwxrwxr-x 8 javierac javierac 4096 Feb  3 10:23 explore
51124142 4 drwxrwxr-x 8 javierac javierac 4096 Feb  3 10:23 .git
51124311 4 drwxrwxr-x 3 javierac javierac 4096 Feb  3 10:23 .github
51124321 4 -rw-rw-r-- 1 javierac javierac   23 Feb  3 10:23 .gitignore
51124322 4 -rw-rw-r-- 1 javierac javierac 1537 Feb  3 10:23 LICENSE
51124382 4 drwxrwxr-x 8 javierac javierac 4096 Feb  3 10:23 map_merge
51124323 8 -rw-rw-r-- 1 javierac javierac 7651 Feb  3 10:23 README.md
javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl/create3_sim_JavAram/create3_ws/src/m-explore-ros2$ cd ../../..
javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl/create3_sim_JavAram$ cd ..
javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl$ git submodule add https://github.com/robo-friends/m-explore-ros2 create3_sim_JavAram/create3_ws/src/m-explore-ros2
Adding existing repo at 'create3_sim_JavAram/create3_ws/src/m-explore-ros2' to the index
javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl$ git commit -m "Add m-explore-ros2 as proper submodule"
[main 6ed8ab8] Add m-explore-ros2 as proper submodule
 2 files changed, 4 insertions(+)
 create mode 100644 .gitmodules
 create mode 160000 create3_sim_JavAram/create3_ws/src/m-explore-ros2
javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl$ git push
Enumerating objects: 12, done.
Counting objects: 100% (12/12), done.
Delta compression using up to 20 threads
Compressing objects: 100% (8/8), done.
Writing objects: 100% (8/8), 846 bytes | 846.00 KiB/s, done.
Total 8 (delta 4), reused 0 (delta 0), pack-reused 0
remote: Resolving deltas: 100% (4/4), completed with 3 local objects.
To github.com:arambarricalvoj/tfm_rl.git
   45c3534..6ed8ab8  main -> main
javierac@localhost:~/Documents/ucm/TFM-PFM/tfm_rl$ 


Para descargar el repo con todos los submodulos:
git clone --recurse-submodules <URL_DEL_REPO>

Actualizar submodules: git submodule update --init --recursive

Actualizar submodules: desde el path del submodule git pull origin main
y desde el repo principal cd ~/Documents/ucm/TFM-PFM/tfm_rl
git add create3_sim_JavAram/create3_ws/src/m-explore-ros2
git commit -m "Update submodule m-explore-ros2"

actualizar todos los submodulos de golpe: git submodule update --remote --merge

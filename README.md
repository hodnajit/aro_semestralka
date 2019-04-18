# ARO sem
Jak postupovat:
* Rozhejbat alepoň trochu robota
* roychodit detekci a naučit neuronku dostatečně dobře!!!!! (nutnuo udělat včas)
* vyzkoušet lokalizaci 
* vyzkoušet exploraci
* Vyzkoušet přesnost 

# Jak rozchodit robota
1.  Připojit se na wifi 
```
SSID:e210bot 
heslo:j6UsAC8a
```
2. Pomocí ssh připojit na robota - * = číslo robota
```
ssh ros@192.168.210.2*
r0sr0s
```
3. Vztvořit workspace a naklonovat složku 
```
mkdir ws_tjl
cd ws_tjl
catkin_init_workspace
mkdir src
cd src
git clone https://gitlab.fel.cvut.cz/rouceto1/aro-sem.git .
```
4. Pracovat a buildit ve složce 
```
catkin build
```
5. po skončení vše smazat
```
cd 
rm -rf ws_tjl
```
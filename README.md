# sze_modellezes_optimalizalas_gyakorlatban
Modellezés és optimalizálás a gyakorlatban (GKNB_INTM019)

<img src="https://miro.medium.com/v2/resize:fit:1200/1*RzxZF0mmXAsMLrIzAWYDSg.png" alt="Jupy" width="300" height="123">

# Hozzáadjuk a PPA-t a stabil verzióval
sudo add-apt-repository ppa:sumo/stable

# Frissítjük a csomaglistát
sudo apt-get update

# Telepítjük a SUMO-t és a szükséges eszközöket
sudo apt-get install sumo sumo-tools sumo-doc

# Beállítjuk a SUMO_HOME környezeti változót
export SUMO_HOME=/usr/bin/sumo

# Frissítjük a PYTHONPATH környezeti változót
export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"

# A Python futtatása a runner.py fájlra
python3 runner.py

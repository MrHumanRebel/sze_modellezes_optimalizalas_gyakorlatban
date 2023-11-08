# sze_modellezes_optimalizalas_gyakorlatban
Modellezés és optimalizálás a gyakorlatban (GKNB_INTM019)


sudo add-apt-repository ppa:sumo/stable

sudo apt-get update

sudo apt-get install sumo sumo-tools sumo-doc

export SUMO_HOME=/usr/bin/sumo

export PYTHONPATH="$SUMO_HOME/tools:$PYTHONPATH"

python3 runner.py
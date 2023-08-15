#!/bin/bash

# Définir le nouveau random_seed souhaité
nouveau_random_seed="12"

path=/Users/fred/CLionProjects/KilobotsExemples/src/examples/experiments/exp2.argos
fname=$(basename "$path") printf $fname


# Vérifier si le fichier exo.argos existe
if [ -f $fname ]; then
    # Utiliser sed pour remplacer la valeur de random_seed par le nouveau_random_seed

  #sed 's/random_seed=\"[0-9]*\"/random_seed=\"$nouveau_random_seed\"/g'
  sed 's/foo/$nouveau_random_seed/g' exp2.argos
  #sed -i "s/random_seed=\"[0-9]*\"/random_seed=\"$nouveau_random_seed\"/" exp2.argos
    echo "La valeur de random_seed a été mise à jour avec $nouveau_random_seed dans exp2.argos."
    argos3 -c /Users/fred/CLionProjects/KilobotsExemples/src/examples/experiments/exp2.argos
else
    echo "Le fichier exp2.argos n'a pas été trouvé."
fi



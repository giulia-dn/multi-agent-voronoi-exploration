**ESPLORAZIONE MULTI-AGENTE DI UN AMBIENTE 2D UTILIZZANDO CELLE DI VORONOI**
-
L'obiettivo del progetto è definire una simulazione multi-agente in cui più robot esplorano uno spazio sconosciuto, aggiornando una mappa condivisa. Ad ogni iterazione, finché l'intera mappa non risulta esplorata:
1) Vengono calcolate le celle di Voronoi per ogni agente.
2) Ogni agente conduce un’esplorazione all’interno della propria cella: il goal viene impostato come il punto di frontiera più vicino all'agente, mentre per il path planning viene implementata una logica RRT.

Per prevenire situazioni di stallo e garantire il corretto partizionamento dello spazio, qualora un agente non disponga di punti di frontiera all'interno della propria cella, il sistema imposta come sub-goal il punto sul confine della cella geometricamente più vicino al punto di frontiera globale più vicino.
Il sistema implementa inoltre logiche di sicurezza per evitare collisioni con gli ostacoli e con gli altri agenti, impedendo che ciascun agente fuoriesca dalla propria cella di competenza durante la navigazione.

**Assunzioni**
-
Gli agenti sono considerati puntiformi, dotati di percezione perfetta a 360° e caratterizzati da una precisione di movimento ideale.
Gli agenti vengono posizionati inizialmente al centro della mappa con una leggera variazione casuale reciproca. Viene inoltre verificato preventivamente che la posizione iniziale di ciascun agente si trovi su uno spazio libero da ostacoli.

**Architettura del Sistema**
-
L'ambiente è modellato come una griglia bidimensionale discreta di dimensioni 500×500 pixel, in cui ogni elemento è codificato tramite una matrice BGR (poiché OpenCV utilizza questo formato anziché RGB): lo spazio esplorato e libero è rappresentato in bianco ([255, 255, 255]), gli ostacoli fissi in nero ([0, 0, 0]) e le aree inesplorate in grigio ([127, 127, 127]).
La mappa è gestita tramite la classe GridWorld, il cui costruttore genera una mappa fissa con ostacoli geometrici. Viene inizializzata una mappa interamente grigia (shared_map), aggiornata dinamicamente dagli agenti durante l'esplorazione. La classe include il metodo final_check, che verifica il completamento della copertura controllando lo stato dei pixel.

La classe Agent modella i singoli robot, inizializzati con posizione e raggio del sensore, e contiene gli attributi per l'albero di esplorazione, il percorso selezionato e la voronoi_mask associata per l'iterazione corrente. I metodi principali di questa classe sono:
- scan_world: per l'aggiornamento della percezione.
- move: per la gestione dello spostamento elementare.
- move_to_goal: per seguire un percorso strutturato a nodi fino al goal attivo.
- step_to_goal: per il calcolo dei nodi intermedi nella costruzione dell'albero RRT.
- path_to_goal: per la ricostruzione a ritroso del percorso sull'albero RRT.
- find_goal: per la selezione dell'obiettivo di esplorazione.

Le funzioni di supporto alla pianificazione comprendono:
- compute_voronoi: calcola le celle di Voronoi globali associando ciascun pixel all'agente più vicino.
- planned_exploration: gestisce la pianificazione di movimento per singolo agente. Invoca find_goal e, se l'obiettivo è molto vicino, salta l'RRT validando il percorso tramite collision_check e muovendosi direttamente con move_to_goal. In caso contrario, utilizza l'algoritmo RRT (impostando una volta su dieci il goal come il punto di campionamento casuale, per guidare la crescita dell'albero verso il goal) l’algoritmo RRT è implementato usando le funzioni di supporto find_closest_node (trova il nodo dell’albero più vicino al punto campionato) e step_to_goal (calcola il nuovo nodo da aggiungere all’albero sul segmento nodo albero più vicino - punto campionato). Se l'RRT fallisce, il goal attivo viene azzerato.
- run_simulation: coordina l'intera simulazione iterativa calcolando le celle di Voronoi, eseguendo la pianificazione per ciascun agente, estraendo le frontiere e chiamando la funzione che gestisce il rendering grafico della scena, render_scene.


**Test di Scalabilità**
-
La funzione run_joint_experiment permette di valutare le prestazioni del sistema al variare del numero di agenti $N \in$ {1,2,4,6,8,10,12,14,16,18,20}. Ciascuna configurazione è stata eseguita per 3 repliche indipendenti per mitigare la varianza stocastica dell'algoritmo RRT.
Le metriche di valutazione adottate sono:
- Iterazioni medie di completamento.
- Distanza totale cumulativa media (somma complessiva della distanza percorsa da tutti gli agenti lungo l'intera simulazione).


*Analisi dei Risultati*

<img src="scalability_and_efficiency_analysis.png" width="800">


- Dal primo grafico (Scalabilità: Agenti vs Iterazioni): L'andamento mostra un decadimento non lineare (simile a una funzione esponenziale inversa) caratterizzato da un crollo verticale delle iterazioni passando da 1 a 4-6 agenti, per poi stabilizzarsi intorno ai 12-14 agenti. Ciò dimostra che l'incremento della flotta apporta benefici significativi grazie alla parallelizzazione, ma oltre la soglia dei 14 agenti la curva tende a un asintoto orizzontale. Ulteriori aggiunte di agenti oltre i 14, risultano superflue poiché la mappa risulta satura e i robot subiscono colli di bottiglia geometrici e interferenze reciproche.

- Dal secondo grafico (Efficienza: Agenti vs Distanza Totale Cumulativa): La distanza totale cresce progressivamente all'aumentare degli agenti fino alla fascia tra 16 e 20 agenti, dove la curva si stabilizza e flette leggermente verso il basso. Fino a 16 agenti, la flotta sostiene un costo energetico e spaziale globale maggiore a causa dell'elevato numero di unità in movimento. La flessione finale nei test a 18-20 agenti evidenzia il fenomeno della saturazione spaziale: lo spazio ristretto riduce le celle di Voronoi a dimensioni tali per cui gli agenti compiono spostamenti minimi o rimangono confinati.

Considerando congiuntamente i due parametri, si conclude che il punto di lavoro ottimale per questo sistema si colloca nell'intervallo tra 10 e 14 agenti, in quanto massimizza il guadagno temporale evitando al contempo un'eccessiva congestione e ridondanza di movimento della flotta.

**ESEMPI DI SIMULAZIONE**
-
**CASI di 2 e 5 agenti**
<p align="center">
  <img src="https://github.com/giulia-dn/multi-agent-voronoi-exploration/releases/download/v1.0/multi_agent_exploration_2_agents.gif" width="45%">
  <img src="https://github.com/giulia-dn/multi-agent-voronoi-exploration/releases/download/v1.0/multi_agent_exploration_5_agents.gif" width="45%">
</p>

**CASI di 10 e 20 agenti**
<p align="center">
  <img src="https://github.com/giulia-dn/multi-agent-voronoi-exploration/releases/download/v1.0/multi_agent_exploration_10_agents.gif" width="45%">
  <img src="img/multi_agent_exploration_20_agents.gif" width="45%">
</p>

Si può notare che il caso con 2 agenti ha bisogno di molte iterazioni, è in realtà caso simile al caso con 1 agente.
La simulazione con 5 agenti si ha una copertura più fluida della mappa.
Si può notare come nel caso di 20 agenti la mappa sia satura di agenti che si muovono in spazi stretti e incorrono in colli di bottiglia geometrici. 
Il caso con 10 agenti continua a mostrare invece, come nel caso con 5 agenti, un comportamento fluido e una buona copertura della mappa.

# wheeled_snake_robots
Exploring robustness and adaptability of locomotion primitives in a wheeled snake robot with self-organized controllers.
---------------------------------------------------------------------------------------------------------
Information on the lpzRobot Simulator can be found at http://robot.informatik.uni-leipzig.de/software/ 

Documentation : http://robot.informatik.uni-leipzig.de/software/doc/html/index.html

******INSTALLATION********

For installation, use the development version from https://github.com/georgmartius/lpzrobots

as a root install is not possible on the university pc, extra directories are created in home. Path variables have therefore to be set manually:
```
export PATH=/home/<your username>/bin:$PATH \\
export LD_LIBRARY_PATH=/home/<your username>/lib
```
then 
```
cd lpzrobots
make all
/home/<your username>
d
Y
```
if the Guilogger does not work:

go into guilogger directory
```
qmake -qt4 guilogger.pro
make
```
then copy the guilogger from guilogger/bin to home/bin

----------------------------------------------------------------------

******USING THE LOGFILES*******

What is written into the logfiles can be adapted in the code. For handling the logfiles when plotting, best use pandas
For the current log files you can use:
```
col = ['t', 
	'phi1', 'phi2', 'phi3', 'phi4', 'phi5', 'phi6', 'phi7', 'phi8', 'phi9', 'phi10',
 	'phidot1', 'phidot2', 'phidot3', 'phidot4', 'phidot5', 'phidot6', 'phidot7', 'phidot8', 'phidot9', 'phidot10',
 	'spring1', 'spring2', 'spring3', 'spring4', 'spring5', 'spring6', 'spring7', 'spring8',
 	'Xdot first', 'Ydot first', 'Xdot last', 'Ydot last',
 	'X first', 'Y first', 'Z first',
 	'motors1', 'motors2', 'motors3', 'motors4', 'motors5', 'motors6', 'motors7', 'motors8', 'motors9', 'motors10', 
	'x_act1', 'x_tar1', 'memb1', 'x_act2', 'x_tar2', 'memb2', 'x_act3', 'x_tar3', 'memb3', 'x_act4', 'x_tar4', 'memb4', 'x_act5', 'x_tar5', 'memb5', 'x_act6', 'x_tar6', 'memb6', 'x_act7', 'x_tar7', 'memb7', 'x_act8', 'x_tar8', 'memb8', 'x_act9', 'x_tar9', 'memb9', 'x_act10', 'x_tar10', 'memb10']

import pandas as pd
data = pd.read_table("Car.log", comment="#", delim_whitespace=True, header=None)
data.columns = col
```
access values:
`data.loc[:,"t"]`

---------------------------------------------------------------------

*******MAKING VIDEOS********

record either in running simulation or check ./start --help for other options

make video out of jpgs:
```
ffmpeg -framerate 24 -i frame_%06d.jpg <filename>.mp4
```

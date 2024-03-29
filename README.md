# blitzBOX V0.1.9 - current stable version<br/>
<br/>
ultra-low-cost and mini-sized ECU for speeduino firmware (Rev0.1.9 45 x 58 mm)<br/>
Integrated Wideband Lambda Controller with Bosch CJ125 and LSU 4.9<br/>
4-Layer highly integrated design - hand soldering possible but only recommended to people with really experienced soldering skills.  Hardware is optimized for assembly by professional assembly manufacturer<br/>
supports 4 cylinder full sequential<br/>

4 high impedance injectors<br/>
4 active ignition coils<br/>
1 crankshaft sensor with MAX9926 VR-conditioner (optional Hall Input without MAX9926)<br/>
1 hall input for camshaft sensor<br/>
6 analog inputs: CLT, IAT, TPS, BAT, LMM, MAP<br/>
1 optional baro sensor MPXH6400A on connector side of pcb<br/>
1 digital input for disco swaggering (aka. launch control)<br/>

Onboard Wideband Lambda Controller with Bosch CJ125 (LSU 4.9)<br/>

1 fuel pump out<br/>
1 idle valve out<br/>
1 tacho out (5/12V)<br/>
1 boost out (optional with IGN_D out)<br/>
1 fan out<br/>

USB programming interface with CP2102<br/>
optional Bluetooth module<br/>

16.09.2020: Betaversion 0.0.6 has gone into production - More infos coming soon....<br/>
30.09.2020: First initial tests of V0.0.6 have been made. Further tests still have to be done. Until now there was no significant or uncorrectable error in V0.0.6. Some minor improvements will lead to next version V0.0.7<br/>
01.01.2021: Happy new year - Version 0.0.8 has been released<br/>
20.05.2021: Betaversion 0.0.9 has been released<br/>
26.05.2021: Betaversion 0.1.0 has been released<br/>
26.08.2021: Betaversion 0.1.1 has been released<br/>
07.12.2021: Betaversion 0.1.2 has been released<br/>
25.04.2022: Betaversion 0.1.2 is running on a real car after two corrections which can be done manually. See correction proposals on issues tab here in GitHub. 
Next and hopefully final version will follow up soon.<br/>
06.05.2022: Final version is getting closer. Also added some information how to wire the connector (interactive page): http://www.oelprinz.org/products/BlitzboxBL49sp/documentation/V0.1.4_and_newer/wiki/pinout_V0.1.4_and_newer_Connector_top_side.html<br/>More information/documentation coming soon<br/>
26.08.2022: release candidate 1 (0.1.3) has been published<br/>
10.11.2022: V0.1.3 became first official released version<br/>
13.02.2023: V0.1.4 minor updates on smd footprints<br/>
13.03.2023: V0.1.5 minor updates<br/>
22.03.2023: V0.1.6 minor updates on power supply - release candidate for final version<br/>
06.04.2023: V0.1.7 added more ground planes - release candidate for final version<br/>
19.04.2023: V0.1.8 added optional Bluetooth module - release candidate for final version<br/>
19.04.2023: V0.1.9 added solder pad for +5V on VR2- (12V Hall Input for CAM) - release candidate for final version<br/>
04.03.2024: V0.1.9 released as current stable version<br/>


[Link to the original speeduino project](https://www.speeduino.com "speeduino homepage")<br/>
<br/>
Quick link to Rev0.1.9 schematic: https://github.com/oelprinz-org/BlitzboxBL49sp/blob/master/hardware/Rev0.1.9/schematics_blitzbox_v0.1.9.pdf <br/>

Quick link to Rev0.1.9 iBOM: http://oelprinz.org/products/BlitzboxBL49sp/hardware/V0.1.9/iBOM/ibom.html <br/>


### Bosch CJ125 (LQFP32 package) part numbers

|Bosch part number|
|:----------------|
|30615|
|40103|
|30522|

### Suitable Bosch lambda probes

Basically, the Bosch probes whose numbers start with **0 258 017** will fit, also Bosch **0 281 004**.  
Bosch numbers beginning with **0 258 007** are *LSU 4.2* probes and will not fit.

|Bosch number  |Length overall|Comment|
|--------------|--------------|-------|
|0 258 017 012 |1060mm||
|0 258 017 025 |1000mm|Bosch motorsport part|
|0 281 004 028 |540mm|common probe for diesel engines|
|0 258 017 029 |620mm|grey, used by BMW after 09/2006 (1178 7539124)|
|0 258 017 038 |340mm|grey, used by BMW (11787537984)|
|0 258 017 092 |950mm|black, used by BMW (1178 7540167)|
|0 258 017 126 |680mm|black, used by BMW after 09/2006 (1178 7561410)|
|0 281 004 150 |1215mm||
|0 281 004 184 |1000mm||
|...|||

### LSU4.9 probe pinout

|Pin#|Color|Description|Symbol|
|----|-----|--------|-----------|
|1|red|Pump current APE|IP|
|2|yellow|Virtual ground IPN|VM|
|3|white|Heater voltage H-|Uh-|
|4|grey|Heater voltaget H+|Uh+|
|5|green|Trim Resistor RT|IA|
|6|black|Nernst voltage UN|RE|


<p align="center">
  <br/>
  <img src="hardware/Rev0.1.9/2022-09-22-Board-Blitzbox-V0.1.3_top.png" width="710" alt="Board-Blitzbox-V0.1.3_top"><br/>
  <img src="hardware/Rev0.1.9/2022-09-22-Board-Blitzbox-V0.1.3_bottom.png" width="710" alt="Board-Blitzbox-V0.1.3_bottom"><br/>
  <img src="hardware/Rev0.1.9/2023-05-15-Board-Blitzbox-V0.1.9.png" width="710" alt="Board-Blitzbox-V0.1.8"><br/>
  <img src="hardware/Rev0.1.9/top.png" width="350" title="top">
  <img src="hardware/Rev0.1.9/bottom.png" width="350" alt="bottom"><br/>
  <img src="hardware/Rev0.1.9/internal_plane1.png" width="350" title="Internal Plane 1">
  <img src="hardware/Rev0.1.9/internal_plane2.png" width="350" title="Internal Plane 2">
  <img src="hardware/Rev0.1.9/top_layer.png" width="350" title="top_layer">
  <img src="hardware/Rev0.1.9/bottom_layer.png" width="350" alt="bottom_layer">
  <img src="hardware/Rev0.1.9/assembly_solderside_V0.1.9.png" width="710" alt="assembly_solderside"><br/>
  <img src="hardware/Rev0.1.9/assembly_connectorside_V0.1.9.png" width="710" alt="assembly_connectorside"><br/>
</p>
<br/>
<p align="center">
  <br/>
</p>

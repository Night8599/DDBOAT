# results/

`logs/` holds the trial logs used in the README and in the report, one trial per mission:

| File | Mission | Trial |
|---|---|---|
| `mission2_016.*` | 2. Crossing and return | 016 |
| `mission3_003.*` | 3. Waypoint route | 003 |
| `mission4_006.*` | 4. Station keeping | 006 |
| `mission5_004.*` | 5. Two-boat crossing | 004 |
| `mission6_002.*` | 6. Buoy capture | 002 |

`.csv` is the 10 Hz log written by the mission script (`x_m`, `y_m` are the Kalman
estimate in the local ENU frame, origin at point B); `_boats.csv` holds the positions
received from the UDP server; `.kml` opens in Google Earth.

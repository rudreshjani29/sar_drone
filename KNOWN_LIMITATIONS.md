# Known Limitations

- **Sortie splitting is naive (Phase 1).** When a search area exceeds one
  battery's coverage budget, the planner currently splits the flattened
  waypoint list into equal-sized chunks rather than clean geographic
  sub-regions. Each sortie can end up spanning the full extent of one
  axis instead of covering a contiguous rectangular patch. Fix planned
  for a later revision: split the search area into N geographic bands up
  front, then independently generate a lawnmower pattern per band.

- **GPS-denial testing is blocked by simulator support (Phase 2).** PX4's
  failure-injection system (`failure gps off`) is only implemented for
  Gazebo Classic, per PX4's own documentation — not the current Gazebo
  (`gz_x500` target) used throughout this project. Commands are accepted
  without error but have no effect; `is_global_position_ok` never goes
  false. Confirmed via a full baseline/dropout/recovery test flight (see
  `phase2-gps-denied-navigation/gps_denial_test.py`) — ground track and
  GPS-health flag showed no change during the intended dropout window.
  Two paths forward, deferred for now: (1) run this specific test under
  Gazebo Classic instead, or (2) simulate GPS loss at a different layer
  (e.g. blocking/corrupting GPS MAVLink messages directly, rather than
  relying on PX4's built-in failure injection).

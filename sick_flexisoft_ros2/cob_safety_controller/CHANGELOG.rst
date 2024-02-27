^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package cob_safety_controller
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.7 (2021-08-02)
------------------
* Merge pull request `#59 <https://github.com/ipa320/sick_flexisoft/issues/59>`_ from benmaidel/feature/magnetic_safety_switch
  handle magnetic safety switch input
* add ack_needed flag
* fix topics queue size
* use same timestamp for all headers
* cleanup update method
* publish safety controller state
* set diagnostics summary to WARN if magnetic_safety_switch is not closed
* do not use magnetic safety switch for emergency state
* do not not use base_active and torso_active flags for emergency_state
* add digital outputs to output struct
* add external_stop_2_ok and magnetic_safety_switch state to diagnostics
* consider magnetic safety switch input in overall em_state
* handle second external_stop input
* correct comm protocol
* handle magnetic safety switch input
* Contributors: Benjamin Maidel, Felix Messmer

0.1.6 (2021-05-10)
------------------
* Merge pull request `#58 <https://github.com/ipa320/sick_flexisoft/issues/58>`_ from fmessmer/fix_catkin_lint
  fix catkin_lint
* fix catkin_lint
* Contributors: Felix Messmer, fmessmer

0.1.5 (2020-10-10)
------------------
* Merge pull request `#57 <https://github.com/ipa320/sick_flexisoft/issues/57>`_ from fmessmer/test_noetic
  test noetic
* Bump CMake version to avoid CMP0048 warning
* Contributors: Felix Messmer, fmessmer

0.1.4 (2020-03-18)
------------------
* Merge pull request `#55 <https://github.com/ipa320/sick_flexisoft/issues/55>`_ from LoyVanBeek/feature/python3_compatibility
  [ci_updates] pylint + Python3 compatibility
* python3 compatibility via 2to3
* Merge pull request `#56 <https://github.com/ipa320/sick_flexisoft/issues/56>`_ from fmessmer/ci_updates
  [travis] ci updates
* use catkin_install_python
* catkin_lint fixes
* Contributors: Felix Messmer, Loy van Beek, fmessmer

0.1.3 (2019-08-14)
------------------
* Merge pull request `#54 <https://github.com/ipa320/sick_flexisoft/issues/54>`_ from fmessmer/fix_loadFile_check
  fix loadFile check
* fix loadFile check
* Contributors: Felix Messmer, fmessmer

0.1.2 (2019-08-12)
------------------
* Merge pull request `#53 <https://github.com/ipa320/sick_flexisoft/issues/53>`_ from fmessmer/melodic_checks
  [Melodic] add melodic checks
* use tinyxml2
* Contributors: Felix Messmer, fmessmer

0.1.1 (2019-03-14)
------------------
* Merge pull request `#52 <https://github.com/ipa320/sick_flexisoft/issues/52>`_ from fmessmer/efi_bus_io_error
  efi bus io error
* move efi_bus_io_error flags
* rearrange diagnostic key value pairs
* rearrange button_stop condition
* add flags for efi_bus_io_error
* add flag for fall sensors released
* Merge pull request `#49 <https://github.com/ipa320/sick_flexisoft/issues/49>`_ from benmaidel/feature/state_valid_param
  enable/disable external state_is_valid validation
* enable/disable external state_is_valid validation
* Merge pull request `#48 <https://github.com/ipa320/sick_flexisoft/issues/48>`_ from benmaidel/improvements
  Reconnect to flexisoft on send failure
* try reconnect to flexi soft on failure
* Merge pull request `#45 <https://github.com/ipa320/sick_flexisoft/issues/45>`_ from ipa-bnm/feature/dirt_detection
  added dirt detection state to diagnostics
* renamed laser dirty diag
* added dirt detetion state to diagnostics
* Merge pull request `#43 <https://github.com/ipa320/sick_flexisoft/issues/43>`_ from ipa-fmw/feature/flexisoft_config_without_wireless_but_with_fall_sensors
  add configuration for flexisoft without wireless and with fall sensors
* add configuration for flexisoft without wireless and with fall sensors
* Merge pull request `#41 <https://github.com/ipa320/sick_flexisoft/issues/41>`_ from ipa-fmw/remove/cob_relayboard
  remove cob_relayboard
* remove cob_relayboard
* Merge pull request `#37 <https://github.com/ipa320/sick_flexisoft/issues/37>`_ from ipa-fmw/fix/flexisoft
  relect mode in em stop topic
* relect mode in em stop topic
* Merge pull request `#36 <https://github.com/ipa320/sick_flexisoft/issues/36>`_ from ipa-fxm/indigo_dev
  missing include
* missing include
* Merge pull request `#33 <https://github.com/ipa320/sick_flexisoft/issues/33>`_ from ipa-fmw/feature/integrate_changes_from_old_wiring
  integrate changes made on indigo_dev_old_wiring branch
* adressed FXM review feedback
* remove debug logoutput
* remove debug output
* remove unused files
* integrate changes made on indigo_dev_old_wiring branch
* Merge pull request `#30 <https://github.com/ipa320/sick_flexisoft/issues/30>`_ from ipa-fmw/indigo_dev
  use small safety fields at startup
* use small safety fields at startup
* Merge pull request `#18 <https://github.com/ipa320/sick_flexisoft/issues/18>`_ from ipa-mdl/indigo_dev
  Extended diagnostics
* include laser status in diagnostics
* laser signals indiate free areas
* Merge pull request `#15 <https://github.com/ipa320/sick_flexisoft/issues/15>`_ from ipa-mdl/indigo_dev
  odometry timeout guard and diagnostics
* tabs -> spaces
* fix diagnostics message, add name and hardware_id, fix tabs vs spaces, add time_since_last_odometry to diagnostics
  Conflicts:
  cob_safety_controller/common/src/safety_controller_common.cpp
* added diagnostics
* switch to fast rotation field if odometry stamp is older than odometry_timeout (default: 1.0)
  fixes `#14 <https://github.com/ipa320/sick_flexisoft/issues/14>`_
* Merge pull request `#12 <https://github.com/ipa320/sick_flexisoft/issues/12>`_ from ipa-mdl/indigo_dev
  Updated cob_safety_controller to new gateway routing
* updated GENT routing
* fixed payload bound checks
* Merge pull request `#11 <https://github.com/ipa320/sick_flexisoft/issues/11>`_ from ipa-fmw/indigo_dev
  add dependency to cob_msgs instead of cob_relayboard
* integrate changes from nhg to BRIDE model
* Merge pull request `#9 <https://github.com/ipa320/sick_flexisoft/issues/9>`_ from ipa320/hydro_dev
  [indigo_dev] added brake_released handling
* Merge pull request `#8 <https://github.com/ipa320/sick_flexisoft/issues/8>`_ from ipa-mdl/hydro_dev
  Bugfix & Brake release support
* added brake_released handling
* memset bug
* indigo_dev
* fix logic for switching safety fields
* Merge pull request `#3 <https://github.com/ipa320/sick_flexisoft/issues/3>`_ from ipa-fmw/hydro_dev
  add metapackage and maintainer info
* change maintainer
* Merge pull request `#2 <https://github.com/ipa320/sick_flexisoft/issues/2>`_ from ipa-mdl/hydro_dev
  added field markers
* added special handling for rotational warn field
* added marker code
* added temporary param parsing
* added tinyxml dependency
* added implementation for laser config parsing / marker filling
* move cob_safety_controller to sick package
* Contributors: Benjamin Maidel, Felix Messmer, Florian Weisshardt, Mathias Lüdtke, Nadia Hammoudeh García, Your Name, eva-bonn, floweisshardt, fmessmer, ipa-cob4-1, ipa-cob4-2, ipa-fmw, ipa-fxm, msh

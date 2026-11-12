# Declared package modularity

Scope: production

| Package | Ca | Ce | Instability | In cycle | Transitive dependents |
|---|---:|---:|---:|---|---:|
| adaptation | 0 | 0 | N/A | False | 0 |
| bsn_interfaces | 4 | 0 | 0.000 | False | 4 |
| central_hub | 0 | 3 | 1.000 | False | 0 |
| patient | 0 | 1 | 1.000 | False | 0 |
| sensor | 0 | 2 | 1.000 | False | 0 |
| shared_components | 1 | 1 | 0.500 | False | 1 |
| system_monitor | 2 | 0 | 0.000 | False | 2 |

Cycles are strongly connected groups, not an enumeration of all paths.
Internal = within scanned root. Counts describe manifests, not verified source or ROS communication.
Transitive dependents exclude the package itself and include direct dependents.

## Cycle groups

None.

## Warnings

bsn_interfaces: member_of_group not expanded into dependency edges

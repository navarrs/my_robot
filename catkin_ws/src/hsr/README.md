# HSR package

This package implements robot service actions. 

### Marker publisher
This node adds/deletes a custom marker to RVIZ via a message request that should contain an action (ADD/DELETE) and a pose. 

A sample script:
```
	./scripts/hsr_marker.zsh
```

In this case, the marker looks like a pink cube:
<p align="center"><img src="./readme/rviz_marker.png" width="600" /></p> 
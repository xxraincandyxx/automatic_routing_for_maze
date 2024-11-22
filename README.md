# automatic_routing_for_maze
This is a solution to maze-like map route planning in dark, that is to say, this would automatically finding a road to a target given limited filed of view(or fog, to games).

As to the algorithm, in fact, it's more like a trick - I just apply the __A*__ algorithm such that it could find the shortest way to the block(or location) of the highest _score_, where the _score_ is due to a heuristic function - which both considering the number of dark fog the target block could dispel and the distance...
It sounds stupid and seems to lack of efficiency, and yes, this is ture. But at least, it works for me. >_<

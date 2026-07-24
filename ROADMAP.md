# Roadmap

## v0.7.0
- [x] Add UV for package manager
- [ ] #7 Add generators of numpy point cloud to make tests and examples. 
- [ ] #17 Add additional output information
- [x] #28 Getting Runtime error when fitting. Division by zero, soved in #33. Verify if need to add to other primitive shapes. 
- [ ] #29 Wrong center for detecting circles, also mentioned in #13 and probably #39
- [ ] #35 Sphere fitting gives cryptic error message if less than 4 points are provided. This could be added also to other primitives. Implement on #36.
- [ ] #40 Cuboid is not cuboid. Rework this primitive in library.
 


## Backlog 

- #16 Adding new cpp wrappers
- #13 Cylinder Fitting has unstable results. We need to find a better algorithm.
- #26 Threading showed good results for 2 or 3 agents. We could re-evalutate the possibility of adding a optional thread parameter in the functions.
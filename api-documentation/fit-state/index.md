# pyransac3d.fit_state

## FitState Objects

```
class FitState(TypedDict)
```

State of a single RANSAC iteration, which every `fit(.)` gives to its optional `callback`.

A callback is useful to plot the fitting progress, inspect intermediate results or implement a custom early-stopping criterion. It is called after every iteration and, if it returns a truthy value, the fit stops early and returns the best result found so far. Iterations which sample degenerate points, like 3 collinear points for a plane, are skipped and never reach the callback.

Treat every array as read-only, because they are the same objects used by the fit.

______________________________________________________________________

#### iteration

Index of the current iteration, starting on 0.

#### sample_indices

Index, in `pts`, of the points sampled on this iteration.

#### sample_points

The points sampled on this iteration, `np.array (S, 3)`, where `S` is how many points the shape needs.

#### model

Candidate shape of this iteration, keyed by the attributes of the shape being fitted, like `center` and `radius` for a `Sphere`.

#### inliers

Index of the points of `pts` which fit the candidate of this iteration.

#### best_model

Best candidate found so far, with the same keys of `model`.

#### best_inliers

Index of the points of `pts` which fit `best_model`.

#### is_best

`True` when the candidate of this iteration became the new `best_model`.

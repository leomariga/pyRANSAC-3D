# Contributing to pyRANSAC_3D
We love your input! We want to make contributing to this project as easy and transparent as possible, whether it's:

- Reporting a bug
- Discussing the current state of the code
- Submitting a fix
- Proposing new features
- Becoming a maintainer

## We Develop with Github
We use github to host code, to track issues and feature requests, as well as accept pull requests.

## Development setup
This project uses [uv](https://docs.astral.sh/uv/) to manage dependencies and the dev environment.

1. [Install uv](https://docs.astral.sh/uv/getting-started/installation/) if you don't have it yet.
2. Clone the repo and sync the environment (this installs the runtime dependencies plus the `dev` dependency group defined in `pyproject.toml`):
   ```sh
   git clone https://github.com/leomariga/pyRANSAC-3D
   cd pyRANSAC-3D
   uv sync
   ```
3. Run project commands with `uv run`, e.g.:
   ```sh
   uv run invoke --list
   ```

## Tests and examples
The `tests` folder has the automated tests, which are run with pytest:
```sh
uv run invoke test
```
They only depend on NumPy: the point clouds are built by `pyransac3d.ShapeGenerator` from the parameters of each shape, so every test checks the fitted shape against the shape it generated, and the whole suite runs in a few seconds without opening any window.

The `examples` folder has the scripts which show how to use the library, one file per shape and per kind of example, so the name says what to expect from it:

| Name | What it does |
| --- | --- |
| `{shape}_simple.py` | The shortest way to call the fitter. Only NumPy, no window, it just prints the parameters it found |
| `{shape}_visual.py` | Fits a shape and plots the result with Open3D, which opens a window |
| `{shape}_animation.py` | Plots every candidate with Open3D while RANSAC is running, using the `callback` of the fitter |

Run them one at a time, e.g.:
```sh
uv run python examples/plane_simple.py
```
There is also `early_stop_callback.py`, which uses the same `callback` to stop the fitting as soon as the result is good enough, instead of plotting it.

## We Use [Github Flow](https://guides.github.com/introduction/flow/index.html), So All Code Changes Happen Through Pull Requests
Pull requests are the best way to propose changes to the codebase (we use [Github Flow](https://guides.github.com/introduction/flow/index.html)). We actively welcome your pull requests:

1. Fork the repo and create a new branch from `master`.
2. If you've added code that should be tested, add tests.
3. Update the documentation.
4. Ensure the test suite passes with `uv run invoke test`.
5. Make sure your code lints.
6. Issue that pull request!
6. Clean unused files before commiting using `uv run invoke clean`

## Any contributions you make will be under the Apache 2.0 Software License
In short, when you submit code changes, your submissions are understood to be under the same [Apache License 2.0](https://www.apache.org/licenses/LICENSE-2.0) that covers the project. Feel free to contact the maintainers if that's a concern.

## Report bugs using Github's [issues](https://github.com/leomariga/pyRANSAC_3D/issues)
We use GitHub issues to track public bugs. Report a bug by [opening a new issue](); it's that easy!

## Write bug reports with detail, background, and sample code
**Great Bug Reports** tend to have:

- A quick summary and/or background
- Steps to reproduce
  - Be specific!
  - Give sample code if you can. Include a sample code that *anyone* can run to reproduce what I was seeing
- What you expected would happen
- What actually happens
- Notes (possibly including why you think this might be happening, or stuff you tried that didn't work)

People *love* thorough bug reports. I'm not even kidding.

## Use a Consistent Coding Style
* 4 spaces for indentation rather than tabs
* We have many interesting commands to help create a better code, try `uv run invoke --list`
* Use `uv run invoke lint` and `uv run invoke format` before commit

## License
By contributing, you agree that your contributions will be licensed under its Apache License 2.0.

## Maintainer Notes
These steps are infrequent, so they're documented here as a reminder.

### Release checklist
1. Pre-release: `uv run invoke test` and `uv run invoke lint`.
2. Bump versions in `pyproject.toml`, `CITATION.cff`, and the README citation block, then `uv lock`.
3. Commit and push to `master`.
4. GitHub Release — tag `v<version>` (triggers the Zenodo archive).
5. Build: `uv build`.
6. Optional: publish to [TestPyPI](https://test.pypi.org) with `uv publish --publish-url https://test.pypi.org/legacy/ --token <test-token>`.
7. PyPI: `uv publish --token pypi-xxxx`.
8. Verify the install from outside the repo:
   ```sh
   uvx --from pyransac3d==<version> python -c "import pyransac3d as pyrsc; print(pyrsc.Plane)"
   ```
9. Docs: `uv run pydoc-markdown` then `uv run mkdocs gh-deploy --force -f build/docs/mkdocs.yml`.
10. Verify [PyPI](https://pypi.org/project/pyransac3d/), [Zenodo](https://zenodo.org/doi/10.5281/zenodo.7212567), and [leomariga.github.io/pyRANSAC-3D](https://leomariga.github.io/pyRANSAC-3D/).

Details for the PyPI and docs steps are below.

### Publishing to PyPI
1. Bump the version in `pyproject.toml` (`[project] version`) and `CITATION.cff` (`version`) — keep them in sync.
2. Build the package:
   ```sh
   uv build
   ```

3. Optional: test the release on [TestPyPI](https://test.pypi.org) first with `uv publish --publish-url https://test.pypi.org/legacy/ --token <test-token>` before publishing for real.

4. Publish (requires a PyPI API token):
   ```sh
   uv publish --token pypi-xxxx
   ```
5. Verify: check [pypi.org/project/pyransac3d](https://pypi.org/project/pyransac3d/) and test-install ephemerally with `uvx` (run from outside the repo, so it doesn't pick up the local `pyransac3d/` source folder):
   ```sh
   uvx --from pyransac3d==<version> python -c "import pyransac3d as pyrsc; print(pyrsc.Plane)"
   ```



### Docstring conventions
Each piece of information is documented in exactly one place, following the same convention as [numpydoc](https://numpydoc.readthedocs.io/en/latest/format.html#documenting-classes), the [Google style guide](https://google.github.io/styleguide/pyguide.html#384-classes) and [scikit-learn](https://scikit-learn.org/stable/developers/develop.html#estimated-attributes):

- Everything a `fit(.)` stores on the object is documented in an `Attributes:` section of the **class** docstring, one line per attribute, and referred to by its plain name (`center`, not `self.center`).
- The `fit(.)` docstring only has its summary, `:param:` for every argument and `:returns:` for what it actually returns. It never repeats the attributes.
- What a `fit(.)` returns is defined by a `NamedTuple` in `pyransac3d/fit_results.py`, with a docstring under each field, so the `:returns:` line only has to name the type and its fields. Being a tuple keeps unpacking working, which is how the library has always been used.
- The `state` dictionary given to a `callback` is documented once, in the `FitState` TypedDict of `pyransac3d/fit_state.py`, with a docstring under each key.

Two caveats of the docs renderer are worth knowing when writing a docstring:

- A docstring which uses `Attributes:` (or any other Google section) has the indentation of every line stripped, so it cannot contain indented markdown like the body of an `!!! note` admonition. Use the `!!! note "The text goes in the title"` form instead.
- The `---` at the end of every docstring is the horizontal rule which separates the entries of a page, so keep it as the last line.

### Generating and publishing docs
Docs are generated from source docstrings via `pydoc-markdown` (config: `pydoc-markdown.yml`) using the `mkdocs` renderer, then deployed to the `gh-pages` branch (served by GitHub Pages).

1. Render the markdown pages and the `mkdocs.yml` into `build/docs`:
   ```sh
   uv run pydoc-markdown
   ```
   To preview the site locally before deploying, add `--build --site-dir _site` (the `--site-dir` is required, `--build` alone fails) or run `uv run mkdocs serve -f build/docs/mkdocs.yml`.
2. Deploy to `gh-pages` (from the repo root; `mkdocs.yml` lives in `build/docs`):
   ```sh
   uv run mkdocs gh-deploy --force -f build/docs/mkdocs.yml
   ```
   `--force` overwrites `gh-pages` with the new build; that branch only ever holds generated site output, so this is expected/safe.
3. Verify at [leomariga.github.io/pyRANSAC-3D](https://leomariga.github.io/pyRANSAC-3D/) (allow a minute or two for GitHub Pages to update).


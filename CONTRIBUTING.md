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

## We Use [Github Flow](https://guides.github.com/introduction/flow/index.html), So All Code Changes Happen Through Pull Requests
Pull requests are the best way to propose changes to the codebase (we use [Github Flow](https://guides.github.com/introduction/flow/index.html)). We actively welcome your pull requests:

1. Fork the repo and create a new branch from `master`.
2. If you've added code that should be tested, add tests.
3. Update the documentation.
4. Ensure the test suite passes.
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


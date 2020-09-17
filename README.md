<!-- <div align="center">
  <img src="https://raw.githubusercontent.com/leomariga/pyTruthTable/master/doc/logo.png"><br>
</div> -->

<!-- -----------------
[![PyPI Latest Release](https://img.shields.io/pypi/v/pyTruthTable.svg?style=for-the-badge)](https://pypi.org/project/pyTruthTable/)
[![License](https://img.shields.io/pypi/l/pyTruthTable.svg?style=for-the-badge)](https://github.com/leomariga/pyTruthTable/blob/master/LICENSE)
 -->
## What is pyRANSAC_3D?
**_pyRANSAC_3D_** is an open source implementation of  .

#### Features:
 - Uses Pandas Dataframe.
 - Big set of logic operations.
 - Simplified function calls.
 - Automatic column naming.
 - Customizible appearence.


## Installation
Requirements: Python 3 and [Pandas](https://github.com/pandas-dev/pandas).

Install with [Pypi](https://pypi.org/project/pyTruthTable/):

```sh
pip3 install pyTruthTable
```

### Take a look: 

<!-- ##### Example 1 - Binary operations

``` python
import pyTruthTable as ptt

# Initialize
t_table = ptt.PyTruthTable(["A", "B"])

# Create relations
t_table.append("not", "A")
t_table.append("and", "A", "B")
t_table.append("or", 2, "B")    # Use the column index or name
t_table.append("xor", -2, -1)
t_table.append("nand", -1, 0, name = "C") # Rename column
t_table.append("equals", "C", "A")
```

|   A   |   B   |  ¬ A  | A ^ B | ¬ A v B | (A ^ B) ⊕ (¬ A v B) |   C  | C ↔ A |
|:-----:|:-----:|:-----:|:-----:|:-------:|:-------------------:|:----:|:-----:|
|  True |  True | False |  True |   True  |        False        | True |  True |
|  True | False | False | False |  False  |        False        | True |  True |
| False |  True |  True | False |   True  |         True        | True | False |
| False | False |  True | False |   True  |         True        | True | False |

##### Example 2 - Prepositional logic clauses

``` python
import pyTruthTable as ptt

# Initialize
tt = ptt.PyTruthTable(["Hot", "Wet", "Rains"])

# Append new column with specified operation
tt.append("and", "Hot", "Wet")
tt.append("implies", 3, "Rains")
```

|  Hot  |  Wet  | Rains | Hot ^ Wet | (Hot ^ Wet) → (Rains) |
|:-----:|:-----:|:-----:|:---------:|:---------------------:|
|  True |  True |  True |    True   |          True         |
|  True |  True | False |    True   |         False         |
|  True | False |  True |   False   |          True         |
|  True | False | False |   False   |          True         |
| False |  True |  True |   False   |          True         |
| False |  True | False |   False   |          True         |
| False | False |  True |   False   |          True         |
| False | False | False |   False   |          True         | -->

### See more examples here


## Documentation & other links
 - The [amazing documentation is this Ṕage](https://leomariga.github.io/pyTruthTable/).
 - Source code in the [Github repository](https://github.com/leomariga/pyTruthTable).
 - [Pypi pakage installer](https://pypi.org/project/pyTruthTable/)


## License
[MIT](https://github.com/leomariga/pyTruthTable/blob/master/LICENSE)

## Contributing is awesome!

See [CONTRIBUTING](https://github.com/leomariga/pyTruthTable/blob/master/CONTRIBUTING.md)




## Contact

Developed with :heart: by [Leonardo Mariga](https://github.com/leomariga) 

leomariga@gmail.com

Did you like it? Remember to click on :star2: button.

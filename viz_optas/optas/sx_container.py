"""! @brief Definition of the SXContainer class."""

import collections
import casadi as cs
import numpy as np
from typing import Union, List, Dict

from .spatialmath import ArrayType


## SX type: https://web.casadi.org/docs/#document-symbolic
SX = cs.casadi.SX

## DM type: https://web.casadi.org/docs/#dm
DM = cs.casadi.DM


class SXContainer(collections.OrderedDict):
    """! Container for SX variables"""

    ## Dict[str, bool]: labels for each item in dict, true means variables are discrete
    is_discrete = {}

    def __add__(self, other):
        """! Add two SXContainer's.

        @param other Another SXContainer instance.
        @return An SXContainer instance containing elements from both this and the other instance.
        """
        assert isinstance(
            other, SXContainer
        ), f"cannot add SXContainer with a variable of type {type(other)}"
        out = SXContainer()
        for label, value in self.items():
            out[label] = value
        for label, value in other.items():
            out[label] = value
        out.is_discrete = {**self.is_discrete, **other.is_discrete}
        return out

    def __setitem__(self, label: str, value: SX) -> None:
        """! Set new SX item.

        @param label Name for the new item.
        @param value An array containing symbolic data.
        """
        assert isinstance(
            value, (SX, float)
        ), f"value must be of type casadi.casadi.SX/float, not {type(value)}"
        if label in self:
            raise KeyError(f"'{label}' already exists")
        super().__setitem__(label, cs.SX(value))
        self.is_discrete[
            label
        ] = False  # assume non-discrete, otherwise variable_is_discrete(..) should be called

    def variable_is_discrete(self, label: str) -> None:
        """! Specify that a given variable is discrete.

        @param label Name for a specific item that appears in the SXContainer instance.
        """
        assert label in self, f"'{label}' was not found"
        self.is_discrete[label] = True

    def has_discrete_variables(self) -> bool:
        """! True if any of the variables in container are discrete, False otherwise.

        @return Boolean indicating if the SXContainer instance contains discrete variables.
        """
        return any(self.is_discrete.values())

    def discrete(self) -> List[bool]:
        """! Returns a list containing discrete classification of each variable.

        @return List of booleans for each variable, the boolean indicates whether the symbolic variable is considered discrete.
        """
        out = []
        for label, value in self.items():
            m, n = value.shape
            out += [self.is_discrete[label]] * (m * n)
        return out

    def vec(self) -> SX:
        """! Vectorize SXContainer.

        @return Array containing the vectorized form for the instance.
        """
        values = list(cs.vec(value) for value in self.values())
        return cs.vertcat(*values)

    def numel(self) -> int:
        """! Return the number of elements.

        @return Number of elements in the SXContainer instance.
        """
        return self.vec().numel()

    def vec2dict(self, vec: Union[DM, np.ndarray, list, tuple]) -> dict:
        """! Convert vector with same layout as SXContainer to dict.

        @param vec Vector assumed to be arranged with same layout as SXContainer instance.
        @return Dictionary containing the reconstructed values.
        """
        vec = cs.DM(vec)  # vec could be np.ndarray or cs.casadi.DM or list/tuple
        out_dict = {}
        for label, value in self.items():
            m, n = value.shape
            mn = m * n
            out_dict[label] = cs.reshape(vec[:mn], m, n)
            vec = vec[mn:]
        return out_dict

    def dict2vec(self, d: Dict[str, ArrayType]) -> Union[DM, SX]:
        """! Vectorize dictionary with same layout as container.

        @param d Dictionary containing values to be vectorized.
        @return Vectorized form for the given dictionary.
        """
        out_vec = cs.DM()
        for label, value in self.items():
            v = d.get(label, cs.DM.zeros(*value.shape))
            out_vec = cs.vertcat(out_vec, cs.vec(v))
        return out_vec

    def zero(self) -> dict:
        """! Return dictionary containing layout of container with zero values.

        @return Dictionary containing only zeros as its values.
        """
        return {label: cs.DM.zeros(*value.shape) for label, value in self.items()}

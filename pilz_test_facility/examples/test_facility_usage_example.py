# Copyright (c) 2019 Pilz GmbH & Co. KG
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import time

from pilz_test_facility.test_facility_manager import TestFacilityManager
from pilz_test_facility.op_modes import OperationMode


def start_program():
    """ Example showing how to set-up robot test facility
        to allow robot motions in option mode T1.
    """
    tfm = TestFacilityManager()
    with tfm:
        tfm.ready_robot_for_motion_in(OperationMode.T1)

        # Perform robot motions...
        time.sleep(2.0)


if __name__ == "__main__":
    start_program()

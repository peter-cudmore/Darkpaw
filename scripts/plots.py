from model import get_active_kinematic_chain
from utilities import *
import matplotlib
matplotlib.use('qt5agg')
import matplotlib.pyplot as plt





def main():
    create_active_transform_animation(2)
    plt.show()


if __name__ == '__main__':
    main()
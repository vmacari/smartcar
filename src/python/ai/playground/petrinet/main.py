from playground.petrinet.PetriNet import PetriNet

if __name__ == '__main__':


    # # combine: 2H + 1O -> 1H2O
    # combineSpec = ("combine", [["H", 2], ["O", 1]], [["H2O", 1]])
    #
    # # split: 1H2O -> 2H + 1O
    # splitSpec = ("split", [["H2O", 1]], [["H", 2], ["O", 1]])
    #
    # petriNet = PetriNet(
    #     ["H", "O", "H2O"],  # species
    #     [combineSpec, splitSpec]  # transitions
    # )
    #
    # initialLabelling = {"H":5, "O":3, "H2O":4}
    # steps = 20
    # petriNet.RunSimulation(steps, initialLabelling)

    birthRule = ("birth", [["Rabbit",1]])
    predatorRule= ("predation", [["Wolf", 1]])

    petriNet = PetriNet(
        ["Rabbit", "Wolf", ""],  # species
        [birthRule, predatorRule]  # transitions
    )

    initialLabelling = {"Rabbit": 5, "Wolf": 3}
    petriNet.RunSimulation(20, initialLabelling)


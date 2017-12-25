from playground.petrinet.Transition import Transition


class PetriNetBase:
    # Fields:
    # speciesNames
    # Transition list
    # labelling: speciesName -&gt; token count

    # constructor
    def __init__(this, speciesNames, transitionSpecs):
        this.speciesNames = speciesNames
        this.transitions = this.BuildTransitions(transitionSpecs)

    def BuildTransitions(this, transitionSpecs):
        transitions = []
        for (transitionName, inputSpecs, outputSpecs) in transitionSpecs:
            transition = Transition(transitionName)
            for degreeSpec in inputSpecs:
                this.SetDegree(transition.inputMap, degreeSpec)
            for degreeSpec in outputSpecs:
                this.SetDegree(transition.outputMap, degreeSpec)
            transitions.append(transition)
        return transitions

    def SetDegree(this, dictionary, degreeSpec):
        speciesName = degreeSpec[0]
        if len(degreeSpec) == 2:
            degree = degreeSpec[1]
        else:
            degree = 1
        dictionary[speciesName] = degree

    def PrintHeader(this):
        #print(string.join(this.speciesNames, ", ") + ", Transition")
        print ("{}, Transition".format(this.speciesNames))

    def PrintLabelling(this):
        for speciesName in this.speciesNames:
            print("{},".format(this.labelling[speciesName]))

            #print(str(this.labelling[speciesName]) + ",")

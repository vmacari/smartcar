class Transition:
    # Fields:
    # transitionName
    # inputMap: speciesName -&gt; inputCount
    # outputMap: speciesName -&gt; outputCount

    # constructor
    def __init__(this, transitionName):
        this.transitionName = transitionName
        this.inputMap = {}
        this.outputMap = {}

    def IsEnabled(this, labelling):
        for inputSpecies in this.inputMap.keys():
            if labelling[inputSpecies] < this.inputMap[inputSpecies]:
                return False  # not enough tokens
        return True  # good to go

    def Fire(this, labelling):

        print(this.transitionName)
        for inputName in this.inputMap.keys():
            labelling[inputName] = labelling[inputName] - this.inputMap[
                inputName]
        for outputName in this.outputMap.keys():
            labelling[outputName] = labelling[outputName] + this.outputMap[
                outputName]

from random import randrange

from playground.petrinet.PetriNetBase import PetriNetBase

class PetriNet(PetriNetBase):
    def RunSimulation(this, iterations, initialLabelling):
        this.PrintHeader()  # prints e.g. "H, O, H2O"
        this.labelling = initialLabelling
        this.PrintLabelling()  # prints e.g. "3, 5, 2"

        for i in range(iterations):
            if this.IsHalted():
                print("halted")
                return
            else:
                this.FireOneRule()
                this.PrintLabelling();
        print("iterations completed")

    def EnabledTransitions(this):
        return filter(lambda transition: transition.IsEnabled(this.labelling),
                      this.transitions)

    def IsHalted(this):
        return len(list(this.EnabledTransitions())) == 0

    def FireOneRule(this):
        this.SelectRandom(this.EnabledTransitions()).Fire(this.labelling)

    def SelectRandom(this, items):
        l = list(items)
        randomIndex = randrange(len(l))
        return l[randomIndex]

function F = SolveAllAerodynamics(SolVariables)

    LowerRotor = CalculateLowerRotor(RotorStates)

    F = LowerRotor.Z_hi1 +LowerRotor.Z_h1



end

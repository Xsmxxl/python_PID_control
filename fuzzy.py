import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from time import sleep
from os import system

system('cls')

# Step 1: Define the fuzzy sets for input variables (cost and benefit)
cost = ctrl.Antecedent(np.arange(0, 11, 1), 'cost')
benefit = ctrl.Antecedent(np.arange(0, 11, 1), 'benefit')

# Membership functions for cost and benefit
cost['low'] = fuzz.trimf(cost.universe, [0, 0, 5])
cost['high'] = fuzz.trimf(cost.universe, [5, 10, 10])
benefit['low'] = fuzz.trimf(benefit.universe, [0, 0, 5])
benefit['high'] = fuzz.trimf(benefit.universe, [5, 10, 10])

# Step 2: Define the fuzzy sets for output variable (cost benefit)
cost_benefit = ctrl.Consequent(np.arange(0, 11, 1), 'cost_benefit')

# Membership functions for cost benefit
cost_benefit['low'] = fuzz.trimf(cost_benefit.universe, [0, 0, 5])
cost_benefit['medium'] = fuzz.trimf(cost_benefit.universe, [3, 5, 7])
cost_benefit['high'] = fuzz.trimf(cost_benefit.universe, [5, 10, 10])

# Step 3: Define the fuzzy rules
rule1 = ctrl.Rule(cost['low'] & benefit['high'], cost_benefit['high'])
rule2 = ctrl.Rule(cost['high'] & benefit['high'], cost_benefit['medium'])
rule3 = ctrl.Rule(cost['low'] & benefit['low'], cost_benefit['low'])
rule4 = ctrl.Rule(cost['high'] & benefit['low'], cost_benefit['low'])

rule1.view()
rule2.view()
rule3.view()

# Step 4: Implement the fuzzy inference system
cost_benefit_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
cost_benefit_sim = ctrl.ControlSystemSimulation(cost_benefit_ctrl)

# Step 5: Test the fuzzy logic system with sample inputs
cost_benefit_sim.input['cost'] = 3  # low cost
cost_benefit_sim.input['benefit'] = 8  # high benefit

cost_benefit_sim.compute()

#cost_benefit['low'].view()
#cost_benefit['medium'].view()
#cost_benefit['high'].view()
cost_benefit.view(sim=cost_benefit_sim)

print("Cost Benefit: ", cost_benefit_sim.output['cost_benefit'])
## Cost Benefit:  7.958333333333334

wait = input("")
# The goal of the project is to simulate a heat source with a temperature of ~600°C
# in porous medium (sandstone). The heat propogation  and the phase change from water to vapor 
# should be simulated. As a starting point, the natural_convection.i and water_vapor_phasechange.i 
# files were used. 
# As of now the heat from the heat souce "heater" does not propagate and the phase change does not work properly.


dbg_actions = true
dbg_residual_norms = true

[Mesh]
  [gen]
    type = GeneratedMeshGenerator
    dim = 2
    nx = 30
    ny = 20
    xmin = 0
    xmax = 300
    ymin = -1000
    ymax = -800
  []
  [heater]
    type = ParsedGenerateSideset
    input = gen
    combinatorial_geometry = 'x > 150 & x < 160 & y = -900'
    new_sideset_name = heater
  []
  uniform_refine = 1
[]

[GlobalParams]
  PorousFlowDictator = dictator
  gravity = '0 -9.81 0'
[]

[Variables]
  [pliq]
    scaling = 1e-7
  []
  [h]
    scaling = 1e-6
  []
[]

[ICs]
  [hic]
    type = PorousFlowFluidPropertyIC
    variable = h
    porepressure = pliq
    property = enthalpy
    temperature = 303.15
    fp = water
  []
  [hydrostatic]
    type = FunctionIC
    variable = pliq
    function = '101325 + (9.81 * 1000 * (-y))'
  []
[]

[BCs]
  [h_bot]
    type = DirichletBC
    variable = h
    value = 2.8e6 
    boundary = heater
  []
  # [h_top]
  #   type = DirichletBC
  #   variable = h
  #   value = 1.2e5 
  #   boundary = top
  # []
  [p_top]
    type = DirichletBC
    variable = pliq
    value = 7.95e06
    boundary = top
  []
[]

[AuxVariables]
  [temperature]
    order = CONSTANT
    family = MONOMIAL
  []
  [density]
    family = MONOMIAL
    order = CONSTANT
  []
[]

[AuxKernels]
  [temperature]
    type = PorousFlowPropertyAux
    variable = temperature
    property = temperature
    execute_on = TIMESTEP_END
  []
  [density]
    type = PorousFlowPropertyAux
    variable = density
    property = density
    execute_on = TIMESTEP_END
  []
[]

[Kernels]
  [mass]
    type = PorousFlowMassTimeDerivative
    variable = pliq
  []
  [massflux]
    type = PorousFlowAdvectiveFlux
    variable = pliq
  []
  [energy]
    type = PorousFlowEnergyTimeDerivative
    variable = h
  []
  [heat_conduction]
    type = PorousFlowHeatConduction
    variable = h
  []
  [heat_advection]
    type = PorousFlowHeatAdvection
    variable = h
  []
[]

[UserObjects]
  [dictator]
    type = PorousFlowDictator
    porous_flow_vars = 'pliq h'
    number_fluid_phases = 2
    number_fluid_components = 1
  []
  [pc]
    type = PorousFlowCapillaryPressureBC
    pe = 1e5
    lambda = 2
    pc_max = 1e6
  []
  [fs]
    type = PorousFlowWaterVapor
    water_fp = water
    capillary_pressure = pc
  []
[]

[Materials]
  [watervapor]
    type = PorousFlowFluidStateSingleComponent
    porepressure = pliq
    enthalpy = h
    capillary_pressure = pc
    fluid_state = fs
  []
  [thermal_conductivity]
    type = PorousFlowThermalConductivityIdeal
    dry_thermal_conductivity = '1 0 0  0 1 0  0 0 1'
    wet_thermal_conductivity = '3 0 0  0 3 0  0 0 3'
  []
  [internal_energy]
    type = PorousFlowMatrixInternalEnergy
    density = 2650
    specific_heat_capacity = 1000
  []
  [permeability]
    type = PorousFlowPermeabilityConst
    permeability = '1e-14 0 0 0 1e-14 0 0 0 1e-14'
  []
  [relperm0]
    type = PorousFlowRelativePermeabilityCorey
    n = 2
    phase = 0
  []
  [relperm1]
    type = PorousFlowRelativePermeabilityCorey
    n = 2
    phase = 1
  []
  [porosity]
    type = PorousFlowPorosityConst
    porosity = 0.2
  []
[]

[FluidProperties]
  [water]
    type = Water97FluidProperties
  []
[]

[Preconditioning]
  active = 'mumps'
  [smp]
    type = SMP
    full = true
    petsc_options_iname = '-ksp_type -pc_type -sub_pc_type -sub_pc_factor_shift_type -pc_asm_overlap'
    petsc_options_value = 'gmres      lu      asm           NONZERO                   2'
  []
  [mumps]
    type = SMP
    full = true
    # Following has been added to solve: Linear solve did not converge due to DIVERGED_PC_FAILED iterations 0 PC failed due to SUBPC_ERROR
    petsc_options = '-snes_converged_reason -ksp_diagonal_scale -ksp_diagonal_scale_fix -ksp_gmres_modifiedgramschmidt -snes_linesearch_monitor'
    petsc_options_iname = '-ksp_type -pc_type -pc_factor_mat_solver_package -pc_factor_shift_type -snes_rtol -snes_atol -snes_max_it'
    petsc_options_value = 'gmres      lu       mumps                         NONZERO               1E-3       1E1       500'
  []
[]

[Executioner]
  type = Transient
  solve_type = NEWTON
  end_time = 63072000
  #dtmax = 1e4
  dtmin = 1e-8
  [TimeStepper]
    type = IterationAdaptiveDT
    dt = 5000
  []
[]

[Postprocessors]
  [enthalpy]
    type = ElementAverageValue
    variable = h
    execute_on = 'initial timestep_end'
  []
  [pliq]
    type = ElementAverageValue
    variable = pliq
    execute_on = 'initial timestep_end'
  []
  [liquid_mass]
    type = PorousFlowFluidMass
    phase = 0
    execute_on = 'initial timestep_end'
  []
  [vapor_mass]
    type = PorousFlowFluidMass
    phase = 1
    execute_on = 'initial timestep_end'
  []
  [liquid_heat]
    type = PorousFlowHeatEnergy
    phase = 0
    execute_on = 'initial timestep_end'
  []
  [vapor_heat]
    type = PorousFlowHeatEnergy
    phase = 1
    execute_on = 'initial timestep_end'
  []
[]

[Debug]
  show_var_residual_norms = ${dbg_residual_norms}
  show_actions=${dbg_actions}
[]

[Outputs]
  exodus = true
[]

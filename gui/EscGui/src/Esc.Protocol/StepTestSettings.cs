namespace Esc.Protocol;

/// <summary>Paso fijo de conmutacion six-step: primera fase en PWM, segunda en LOW, tercera flotante.</summary>
public sealed record StepTestSettings(CommutationStep Step, double DutyPercent);

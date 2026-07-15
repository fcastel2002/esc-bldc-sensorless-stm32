using Esc.Bridge;

namespace Esc.Tests;

public sealed class RpmPiReferenceTests
{
    [Fact]
    public void StepUsesRpmErrorAndCanonicalArrScaling()
    {
        var model = new RpmPiReferenceModel();
        ValidationReferenceConfig config = Config(pwmArr: 2000);

        ushort pwm = model.Step(0, 1000, config);

        Assert.Equal((ushort)280, pwm);
    }

    [Fact]
    public void StepScalesCanonicalOutputToActiveArr()
    {
        var model = new RpmPiReferenceModel();

        ushort pwm = model.Step(0, 1000, Config(pwmArr: 1000));

        Assert.Equal((ushort)140, pwm);
    }

    [Fact]
    public void MinimumOutputIsFivePercentAtZeroError()
    {
        var model = new RpmPiReferenceModel();

        ushort pwm = model.Step(1000, 1000, Config(pwmArr: 2000));

        Assert.Equal((ushort)100, pwm);
    }

    private static ValidationReferenceConfig Config(int pwmArr) => new(
        0.28,
        1.00,
        0,
        18_000,
        2,
        pwmArr,
        0.002,
        AlgorithmVersion: 2);
}

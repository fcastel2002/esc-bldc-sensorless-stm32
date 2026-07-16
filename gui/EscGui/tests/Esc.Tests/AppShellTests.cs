namespace Esc.Tests;

public sealed class AppShellTests
{
    [Fact]
    public void InteractiveHeadOutletPrecedesStableHeadResources()
    {
        string appMarkup = File.ReadAllText(
            Path.Combine(AppContext.BaseDirectory, "TestAssets", "App.razor"));
        int headOutletIndex = appMarkup.IndexOf("<HeadOutlet", StringComparison.Ordinal);

        Assert.True(headOutletIndex >= 0, "App.razor must contain an interactive HeadOutlet.");

        string[] trailingResources =
        [
            "<ResourcePreloader",
            "<link rel=\"stylesheet\"",
            "<ImportMap"
        ];

        foreach (string resource in trailingResources)
        {
            int resourceIndex = appMarkup.IndexOf(resource, StringComparison.Ordinal);
            Assert.True(
                resourceIndex > headOutletIndex,
                $"{resource} must remain after HeadOutlet so browser DOM mutations cannot detach Blazor component 1.");
        }
    }
}

using System.Diagnostics;
using Esc.Bridge;
using Esc.Web.Components;
using Esc.Web.Services;
using Microsoft.AspNetCore.Hosting.Server;
using Microsoft.AspNetCore.Hosting.Server.Features;

var builder = WebApplication.CreateBuilder(args);

// Add services to the container.
builder.Services.AddRazorComponents()
    .AddInteractiveServerComponents();
builder.Services.AddEscBridge();
builder.Services.AddSingleton<ModularControlProvider>();

var app = builder.Build();
bool openBrowserOnStartup = builder.Configuration.GetValue(
    "GuiLauncher:OpenBrowserOnStartup",
    !app.Environment.IsDevelopment());

// Configure the HTTP request pipeline.
if (!app.Environment.IsDevelopment())
{
    app.UseExceptionHandler("/Error", createScopeForErrors: true);
}
app.UseStatusCodePagesWithReExecute("/not-found", createScopeForStatusCodePages: true);
app.UseWebSockets();
app.UseAntiforgery();

app.MapStaticAssets();
app.MapEscBridgeEndpoints();
app.MapRazorComponents<App>()
    .AddInteractiveServerRenderMode();

if (openBrowserOnStartup)
{
    app.Lifetime.ApplicationStarted.Register(() =>
    {
        string? address = app.Services
            .GetRequiredService<IServer>()
            .Features
            .Get<IServerAddressesFeature>()?
            .Addresses
            .FirstOrDefault();

        if (string.IsNullOrWhiteSpace(address))
        {
            return;
        }

        try
        {
            Process.Start(new ProcessStartInfo
            {
                FileName = address,
                UseShellExecute = true
            });
        }
        catch
        {
            // Si no se puede abrir el navegador, el servidor igual queda levantado.
        }
    });
}

app.Run();

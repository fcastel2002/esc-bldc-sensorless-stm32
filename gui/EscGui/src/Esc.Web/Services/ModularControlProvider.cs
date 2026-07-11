using System.Text.Json;

namespace Esc.Web.Services;

public sealed class ModularControlProvider
{
    private readonly IWebHostEnvironment _environment;
    private readonly ILogger<ModularControlProvider> _logger;
    private IReadOnlyList<DashboardControl>? _cachedControls;

    public string? LastError { get; private set; }

    public ModularControlProvider(IWebHostEnvironment environment, ILogger<ModularControlProvider> logger)
    {
        _environment = environment;
        _logger = logger;
    }

    public async Task<IReadOnlyList<DashboardControl>> GetControlsAsync(CancellationToken cancellationToken = default)
    {
        if (_cachedControls is not null)
        {
            return _cachedControls;
        }

        string path = Path.Combine(_environment.WebRootPath, "controls.json");
        if (!File.Exists(path))
        {
            LastError = "El archivo controls.json no existe en wwwroot.";
            _cachedControls = Array.Empty<DashboardControl>();
            return _cachedControls;
        }

        try
        {
            await using FileStream stream = File.OpenRead(path);
            DashboardControl[]? controls = await JsonSerializer.DeserializeAsync<DashboardControl[]>(
                stream,
                new JsonSerializerOptions(JsonSerializerDefaults.Web),
                cancellationToken);

            LastError = null;
            _cachedControls = controls ?? Array.Empty<DashboardControl>();
        }
        catch (Exception ex)
        {
            _logger.LogWarning(ex, "Could not load modular controls from {Path}.", path);
            LastError = $"No se pudieron cargar los controles de controls.json: {ex.Message}";
            _cachedControls = Array.Empty<DashboardControl>();
        }

        return _cachedControls;
    }
}

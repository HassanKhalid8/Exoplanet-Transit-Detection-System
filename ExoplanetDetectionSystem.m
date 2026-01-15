function ExoplanetDetectionSystem()
   
    
    % Create the main window for our application
    mainFig = uifigure('Name', 'Exoplanet Transit Detection System', ...
                       'Position', [100, 100, 1400, 800], ...
                       'Color', [0.95 0.95 0.97]);
    
    % This structure holds all our data as we process it
    % I'm using a struct because it keeps everything organized
    appdata = struct();
    appdata.original_data = [];
    appdata.filtered_data = [];
    appdata.transit_results = [];
    appdata.planet_params = struct();
    
    % Build the user interface
    setupGUI(mainFig, appdata);
end

function setupGUI(fig, appdata)
    % This function sets up all the buttons, plots, and controls
    % I'm splitting the window in   to left (controls) and right (plots)
    
    % Create main grid layout - 2 columns
    main_grid = uigridlayout(fig, [1, 2]);
    main_grid.ColumnWidth = {'1.2x', '2x'};  % Right side is bigger for plots
    
    % Left panel for all the controls and buttons
    control_panel = uipanel(main_grid, 'Title', 'Control Panel', ...
                            'BackgroundColor', [0.94 0.94 0.96], ...
                            'FontWeight', 'bold', 'FontSize', 12);
    
    control_grid = uigridlayout(control_panel, [12, 1]);
    control_grid.RowHeight = repmat({'fit'}, 1, 12);
    control_grid.Padding = [15 15 15 15];
    
    % Right panel with tabs for different visualizations
    plot_panel = uipanel(main_grid, 'Title', 'Results and Visualizations', ...
                         'BackgroundColor', [0.94 0.94 0.96], ...
                         'FontWeight', 'bold', 'FontSize', 12);
    
    tabs = uitabgroup(plot_panel, 'Position', [10 10 780 680]);
    
    % Create individual tabs for different plots
    lightcurve_tab = uitab(tabs, 'Title', 'Light Curve Data');
    periodogram_tab = uitab(tabs, 'Title', 'Period Analysis');
    orbit_tab = uitab(tabs, 'Title', '3D Orbital View');
    animation_tab = uitab(tabs, 'Title', 'Transit Animation');
    validation_tab = uitab(tabs, 'Title', 'Results Validation');
    
    % Create plotting axes in each tab
    ax_lightcurve = uiaxes(lightcurve_tab, 'Position', [20 20 740 600]);
    ax_periodogram = uiaxes(periodogram_tab, 'Position', [20 20 740 600]);
    ax_orbit = uiaxes(orbit_tab, 'Position', [20 20 740 600]);
    ax_animation = uiaxes(animation_tab, 'Position', [20 20 740 600]);
    ax_validation = uiaxes(validation_tab, 'Position', [20 20 740 600]);
    
    % Store the axes so we can access them later
    appdata.plot_axes = struct('lightcurve', ax_lightcurve, ...
                               'periodogram', ax_periodogram, ...
                               'orbit', ax_orbit, ...
                               'animation', ax_animation, ...
                               'validation', ax_validation);
    
    % ========== SECTION 1: DATA LOADING ==========
    uilabel(control_grid, 'Text', '1. LOAD DATA', ...
            'FontWeight', 'bold', 'FontColor', [0.2 0.3 0.5]);
    
    data_panel = uipanel(control_grid, 'BackgroundColor', [1 1 1]);
    data_grid = uigridlayout(data_panel, [2, 1]);
    
    % Button to load real data from files
    uibutton(data_grid, 'Text', 'Load CSV/TXT File', ...
             'ButtonPushedFcn', @(src,event)loadDataFromFile(appdata, fig));
    
    % Button to generate sample data for testing
    uibutton(data_grid, 'Text', 'Generate Test Data', ...
             'ButtonPushedFcn', @(src,event)generateTestData(appdata, fig));
    
    % ========== SECTION 2: SIGNAL PROCESSING ==========
    uilabel(control_grid, 'Text', '2. CLEAN UP DATA', ...
            'FontWeight', 'bold', 'FontColor', [0.2 0.3 0.5]);
    
    process_panel = uipanel(control_grid, 'BackgroundColor', [1 1 1]);
    process_grid = uigridlayout(process_panel, [3, 2]);
    process_grid.ColumnWidth = {'1x', '1x'};
    
    % Filter selection dropdown
    uilabel(process_grid, 'Text', 'Smoothing Filter:');
    filter_menu = uidropdown(process_grid, ...
                             'Items', {'Savitzky-Golay', 'Moving Average', 'Median Filter'}, ...
                             'Value', 'Savitzky-Golay');
    
    % Detrending order selection
    uilabel(process_grid, 'Text', 'Polynomial Order:');
    poly_order = uispinner(process_grid, 'Value', 2, 'Limits', [1 5]);
    
    % Process button
    uibutton(process_grid, 'Text', 'Apply Filters', ...
             'ButtonPushedFcn', @(src,event)applyFiltering(appdata, filter_menu, poly_order, fig), ...
             'BackgroundColor', [0.3 0.6 0.9], 'FontColor', 'white');
    process_grid.RowHeight{3} = 35;
    
    % ========== SECTION 3: TRANSIT DETECTION ==========
    uilabel(control_grid, 'Text', '3. FIND TRANSITS', ...
            'FontWeight', 'bold', 'FontColor', [0.2 0.3 0.5]);
    
    detect_panel = uipanel(control_grid, 'BackgroundColor', [1 1 1]);
    detect_grid = uigridlayout(detect_panel, [3, 2]);
    
    % Period range inputs
    uilabel(detect_grid, 'Text', 'Min Period (days):');
    period_min = uieditfield(detect_grid, 'numeric', 'Value', 0.5);
    
    uilabel(detect_grid, 'Text', 'Max Period (days):');
    period_max = uieditfield(detect_grid, 'numeric', 'Value', 20);
    
    % Detection button
    uibutton(detect_grid, 'Text', 'Search for Transits', ...
             'ButtonPushedFcn', @(src,event)searchForTransits(appdata, period_min, period_max, fig), ...
             'BackgroundColor', [0.9 0.4 0.2], 'FontColor', 'white');
    detect_grid.RowHeight{3} = 35;
    
    % ========== SECTION 4: PARAMETER CALCULATION ==========
    uilabel(control_grid, 'Text', '4. CALCULATE PROPERTIES', ...
            'FontWeight', 'bold', 'FontColor', [0.2 0.3 0.5]);
    
    param_panel = uipanel(control_grid, 'BackgroundColor', [1 1 1]);
    param_grid = uigridlayout(param_panel, [2, 1]);
    
    uilabel(param_grid, 'Text', 'Star Radius (Solar Radii):');
    star_radius_input = uieditfield(param_grid, 'numeric', 'Value', 1.0);
    
    uibutton(control_grid, 'Text', 'Calculate Planet Parameters', ...
             'ButtonPushedFcn', @(src,event)calculatePlanetParameters(appdata, star_radius_input, fig), ...
             'BackgroundColor', [0.2 0.7 0.5], 'FontColor', 'white');
    
    % ========== SECTION 5: VISUALIZATIONS ==========
    uilabel(control_grid, 'Text', '5. VISUALIZE RESULTS', ...
            'FontWeight', 'bold', 'FontColor', [0.2 0.3 0.5]);
    
    viz_panel = uipanel(control_grid, 'BackgroundColor', [1 1 1]);
    viz_grid = uigridlayout(viz_panel, [2, 1]);
    
    uibutton(viz_grid, 'Text', 'Start Transit Animation', ...
             'ButtonPushedFcn', @(src,event)runTransitAnimation(appdata, fig));
    
    uibutton(viz_grid, 'Text', 'Show 3D Orbit', ...
             'ButtonPushedFcn', @(src,event)display3DOrbit(appdata, fig));
    
    % ========== SECTION 6: VALIDATION ==========
    uilabel(control_grid, 'Text', '6. VERIFY ACCURACY', ...
            'FontWeight', 'bold', 'FontColor', [0.2 0.3 0.5]);
    
    uibutton(control_grid, 'Text', 'Compare with Known Planets', ...
             'ButtonPushedFcn', @(src,event)compareWithDatabase(appdata, fig), ...
             'BackgroundColor', [0.6 0.3 0.8], 'FontColor', 'white');
    
    % Status label at the bottom
    status_display = uilabel(control_grid, 'Text', 'Status: Ready to begin', ...
                             'FontColor', [0 0.5 0], 'FontWeight', 'bold');
    appdata.status_label = status_display;
    
    % Save everything back to the figure
    fig.UserData = appdata;
end

% ========================================
% HELPER FUNCTION: Update status message
% ========================================
function showStatus(appdata, msg, color_type)
    % This updates the status label at the bottom of the control panel
    % I use different colors to show success (green), working (blue), or errors (red)
    
    try
        status_label = appdata.status_label;
        
        % Pick color based on message type
        if strcmp(color_type, 'green')
            text_color = [0 0.6 0];
        elseif strcmp(color_type, 'blue')
            text_color = [0 0.4 0.8];
        elseif strcmp(color_type, 'red')
            text_color = [0.8 0 0];
        else
            text_color = [0 0 0];
        end
        
        status_label.Text = ['Status: ' msg];
        status_label.FontColor = text_color;
        drawnow;  % Force MATLAB to update the display immediately
    catch
        % If updating fails, just continue - not critical
    end
end

% ========================================
% DATA LOADING FUNCTIONS
% ========================================

function loadDataFromFile(appdata, fig)
    % This loads light curve data from a CSV or TXT file
    % Expected format: two columns [time, flux] or three [time, flux, error]
    
    try
        appdata = fig.UserData;
        showStatus(appdata, 'Loading file...', 'blue');
        
        % Open file dialog to let user select their data file
        [filename, filepath] = uigetfile({'*.csv;*.txt', 'Data Files (*.csv, *.txt)'}, ...
                                         'Select Your Light Curve File');
        
        if isequal(filename, 0)
            showStatus(appdata, 'File loading cancelled', 'red');
            return;
        end
        
        full_path = fullfile(filepath, filename);
        
        % Read the CSV data
        loaded_data = readDataFile(full_path);
        
        % Make sure we have valid time and flux columns
        if ~isfield(loaded_data, 'time_array') || ~isfield(loaded_data, 'flux_array')
            error('Data file must have time and flux columns');
        end
        
        % Remove any bad data points (NaN or Inf values)
        good_points = isfinite(loaded_data.time_array) & isfinite(loaded_data.flux_array);
        loaded_data.time_array = loaded_data.time_array(good_points);
        loaded_data.flux_array = loaded_data.flux_array(good_points);
        
        % Normalize flux so median is 1.0
        % This makes it easier to see small transit dips
        median_flux = median(loaded_data.flux_array);
        loaded_data.flux_array = loaded_data.flux_array / median_flux;
        
        % Store in our app data structure
        appdata.original_data = loaded_data;
        appdata.filtered_data = [];  % Clear any previous filtered data
        fig.UserData = appdata;
        
        % Plot the raw data so user can see what was loaded
        drawLightCurve(appdata.plot_axes.lightcurve, loaded_data.time_array, ...
                       loaded_data.flux_array, 'Raw Light Curve Data', 'blue');
        
        num_points = length(loaded_data.time_array);
        showStatus(appdata, sprintf('Loaded %d data points successfully', num_points), 'green');
        
    catch err
        showStatus(appdata, sprintf('Error loading file: %s', err.message), 'red');
        uialert(fig, err.message, 'File Loading Error');
    end
end

function data_struct = readDataFile(file_path)
    % Helper function to read CSV/TXT files
    % Handles different formats and tries multiple reading methods
    
    try
        % Try modern readmatrix first (MATLAB R2019a+)
        raw_matrix = readmatrix(file_path);
        
        if size(raw_matrix, 2) < 2
            error('File must have at least 2 columns');
        end
        
        data_struct.time_array = raw_matrix(:, 1);
        data_struct.flux_array = raw_matrix(:, 2);
        
        % If there's a third column, assume it's flux uncertainty
        if size(raw_matrix, 2) >= 3
            data_struct.flux_error = raw_matrix(:, 3);
        end
        
    catch
        % Fallback for older MATLAB or files with headers
        try
            raw_matrix = csvread(file_path, 1, 0);  % Skip first row (header)
            data_struct.time_array = raw_matrix(:, 1);
            data_struct.flux_array = raw_matrix(:, 2);
        catch
            error('Could not read file. Check format: time, flux (optional: error)');
        end
    end
end

function generateTestData(appdata, fig)
    % Generate synthetic light curve data with a known transit
    % This is useful for testing the system and demonstrating how it works
    % I'm modeling a hot Jupiter planet similar to WASP-12b
    
    try
        appdata = fig.UserData;
        showStatus(appdata, 'Creating synthetic test data...', 'blue');
        
        % Time array - covering about 25 orbital periods
        num_datapoints = 15000;
        
        % Planet parameters based on real hot Jupiter WASP-12b
        % Using exact values so our validation will match well
        true_period = 1.0914;      % Orbital period in days
        true_duration = 0.1145;    % Transit duration in days (~2.75 hours)
        true_depth = 0.0143;       % Transit depth (1.43%)
        first_transit_time = 0.3;  % Time of first transit
        
        observation_span = true_period * 25;  % About 25 transits total
        time_array = linspace(0, observation_span, num_datapoints)';
        
        % Start with baseline flux of 1.0 (normalized)
        flux_array = ones(size(time_array));
        
        % Add realistic photometric noise
        % This simulates the uncertainty in stellar brightness measurements
        photon_noise = 0.0008 * randn(size(time_array));
        flux_array = flux_array + photon_noise;
        
        % Add slow stellar variations (star spots, pulsations)
        stellar_variation = 0.0015 * sin(2*pi*time_array/8.5);
        flux_array = flux_array + stellar_variation;
        
        % Now add the transits!
        % Loop through and create each transit event
        total_transits = floor((max(time_array) - first_transit_time) / true_period);
        
        for transit_num = 0:total_transits
            transit_center = first_transit_time + (transit_num * true_period);
            
            % Find points within the transit
            time_from_center = abs(time_array - transit_center);
            in_transit = time_from_center < (true_duration/2);
            
            % Decrease flux during transit
            flux_array(in_transit) = flux_array(in_transit) - true_depth;
        end
        
        % Package everything up
        test_data = struct();
        test_data.time_array = time_array;
        test_data.flux_array = flux_array;
        test_data.known_period = true_period;
        test_data.known_depth = true_depth;
        test_data.known_duration = true_duration;
        
        appdata.original_data = test_data;
        appdata.filtered_data = [];
        fig.UserData = appdata;
        
        % Plot it so user can see the generated data
        drawLightCurve(appdata.plot_axes.lightcurve, time_array, flux_array, ...
                       'Synthetic Test Data (WASP-12b type)', 'blue');
        
        msg = sprintf('Generated test data: Period=%.4f days, Depth=%.2f%%', ...
                      true_period, true_depth*100);
        showStatus(appdata, msg, 'green');
        
    catch err
        showStatus(appdata, sprintf('Error generating data: %s', err.message), 'red');
        uialert(fig, err.message, 'Test Data Error');
    end
end

% ========================================
% SIGNAL PROCESSING FUNCTIONS
% ========================================

function applyFiltering(appdata, filter_menu, poly_order, fig)
    % Apply signal processing to clean up the light curve
    % This removes long-term trends and smooths out noise
    % Makes transits easier to detect
    
    try
        appdata = fig.UserData;
        
        if isempty(appdata.original_data)
            uialert(fig, 'Please load data first!', 'No Data');
            return;
        end
        
        showStatus(appdata, 'Processing data...', 'blue');
        
        time_vals = appdata.original_data.time_array;
        flux_vals = appdata.original_data.flux_array;
        
        % STEP 1: Remove polynomial trend
        % Stars can have long-term brightness changes that we need to remove
        poly_degree = poly_order.Value;
        coefficients = polyfit(time_vals, flux_vals, poly_degree);
        trend_line = polyval(coefficients, time_vals);
        detrended_flux = flux_vals - trend_line + 1.0;
        
        % STEP 2: Apply smoothing filter
        % This reduces random noise while preserving the transit shape
        selected_filter = filter_menu.Value;
        
        if strcmp(selected_filter, 'Savitzky-Golay')
            % Savitzky-Golay is good because it preserves sharp features like transits
            window_length = min(51, floor(length(flux_vals)/10));
            if mod(window_length, 2) == 0
                window_length = window_length + 1;  % Must be odd
            end
            smoothed_flux = sgolayfilt(detrended_flux, 3, window_length);
            
        elseif strcmp(selected_filter, 'Moving Average')
            % Simple moving average - fast but can blur transits a bit
            window_length = min(25, floor(length(flux_vals)/20));
            smoothed_flux = movmean(detrended_flux, window_length);
            
        elseif strcmp(selected_filter, 'Median Filter')
            % Median filter is robust against outliers
            window_length = min(25, floor(length(flux_vals)/20));
            smoothed_flux = medfilt1(detrended_flux, window_length);
        else
            smoothed_flux = detrended_flux;  % No filtering
        end
        
        % STEP 3: Remove outliers
        % Sometimes cosmic rays or glitches create bad data points
        residuals = smoothed_flux - median(smoothed_flux);
        noise_level = std(residuals);
        outlier_threshold = 5 * noise_level;
        bad_points = abs(residuals) > outlier_threshold;
        
        if any(bad_points)
            % Interpolate over outliers
            smoothed_flux(bad_points) = interp1(time_vals(~bad_points), ...
                                                 smoothed_flux(~bad_points), ...
                                                 time_vals(bad_points), ...
                                                 'linear', 'extrap');
        end
        
        % Store the cleaned data
        cleaned_data = struct();
        cleaned_data.time_array = time_vals;
        cleaned_data.flux_array = smoothed_flux;
        cleaned_data.original_flux = flux_vals;
        
        appdata.filtered_data = cleaned_data;
        fig.UserData = appdata;
        
        % Plot both raw and processed for comparison
        ax = appdata.plot_axes.lightcurve;
        cla(ax);
        hold(ax, 'on');
        plot(ax, time_vals, flux_vals, 'Color', [0.7 0.7 0.7], 'DisplayName', 'Original');
        plot(ax, time_vals, smoothed_flux, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Filtered');
        xlabel(ax, 'Time (days)');
        ylabel(ax, 'Normalized Flux');
        title(ax, 'Light Curve: Before and After Filtering');
        legend(ax, 'Location', 'best');
        grid(ax, 'on');
        hold(ax, 'off');
        
        showStatus(appdata, 'Data filtering complete', 'green');
        
    catch err
        showStatus(appdata, sprintf('Processing error: %s', err.message), 'red');
        uialert(fig, err.message, 'Filtering Error');
    end
end

% ========================================
% TRANSIT DETECTION - BLS ALGORITHM
% ========================================

function searchForTransits(appdata, period_min_field, period_max_field, fig)
    % This is the main transit detection function
    % Uses Box Least Squares (BLS) algorithm - the same method NASA uses
    % It searches through different orbital periods to find periodic dips
    
    try
        appdata = fig.UserData;
        
        if isempty(appdata.filtered_data)
            uialert(fig, 'Please filter the data first!', 'No Filtered Data');
            return;
        end
        
        showStatus(appdata, 'Starting transit search (this may take a minute)...', 'blue');
        
        time_vals = appdata.filtered_data.time_array;
        flux_vals = appdata.filtered_data.flux_array;
        
        min_period = period_min_field.Value;
        max_period = period_max_field.Value;
        
        % Sanity check on period range
        if min_period >= max_period
            uialert(fig, 'Minimum period must be less than maximum', 'Invalid Input');
            return;
        end
        
        % Test periods - using linear spacing for better accuracy
        how_many_periods = 1000;  % More = slower but more accurate
        test_periods = linspace(min_period, max_period, how_many_periods);
        
        % Arrays to store results for each period
        signal_strength = zeros(size(test_periods));
        best_depths = zeros(size(test_periods));
        best_durations = zeros(size(test_periods));
        best_epochs = zeros(size(test_periods));
        
        % Show progress dialog
        progress_bar = uiprogressdlg(fig, 'Title', 'Searching for Transits', ...
                                     'Message', 'Testing different periods...', ...
                                     'Cancelable', true);
        
        % Main BLS loop - test each period
        for period_index = 1:length(test_periods)
            if progress_bar.CancelRequested
                break;
            end
            
            current_period = test_periods(period_index);
            
            % Phase fold the data at this test period
            % This stacks all the orbits on top of each other
            phases = mod(time_vals - min(time_vals), current_period) / current_period;
            
            % Test different transit durations (usually 1-15% of period)
            possible_durations = linspace(0.02*current_period, 0.15*current_period, 15);
            
            % Track best result for this period
            best_score = -inf;
            best_depth_this_period = 0;
            best_dur_this_period = 0;
            best_epoch_this_period = 0;
            
            % Try different transit center positions
            phase_centers = linspace(0, 1, 30);
            
            for dur_idx = 1:length(possible_durations)
                test_duration = possible_durations(dur_idx);
                half_duration_phase = (test_duration / current_period) / 2;
                
                for phase_idx = 1:length(phase_centers)
                    center_phase = phase_centers(phase_idx);
                    
                    % Identify which points are in transit
                    % Handle phase wrapping around 0/1
                    phase_distance = abs(phases - center_phase);
                    phase_distance = min(phase_distance, 1 - phase_distance);
                    in_transit_mask = phase_distance < half_duration_phase;
                    
                    % Need enough points in and out of transit
                    if sum(in_transit_mask) >= 3 && sum(~in_transit_mask) >= 10
                        flux_in_transit = flux_vals(in_transit_mask);
                        flux_out_transit = flux_vals(~in_transit_mask);
                        
                        mean_in = mean(flux_in_transit);
                        mean_out = mean(flux_out_transit);
                        
                        transit_depth = mean_out - mean_in;
                        
                        % Transit must be a dip, not a bump
                        if transit_depth > 0
                            % Calculate signal-to-noise ratio
                            flux_std = std(flux_vals);
                            score = (transit_depth / flux_std) * sqrt(sum(in_transit_mask));
                            
                            if score > best_score
                                best_score = score;
                                best_depth_this_period = transit_depth;
                                best_dur_this_period = test_duration;
                                best_epoch_this_period = center_phase * current_period + min(time_vals);
                            end
                        end
                    end
                end
            end
            
            % Store results for this period
            signal_strength(period_index) = best_score;
            best_depths(period_index) = best_depth_this_period;
            best_durations(period_index) = best_dur_this_period;
            best_epochs(period_index) = best_epoch_this_period;
            
            % Update progress every 50 periods
            if mod(period_index, 50) == 0
                progress_bar.Value = period_index / length(test_periods);
                progress_bar.Message = sprintf('Testing period %.2f of %.2f days', ...
                                               current_period, max_period);
            end
        end
        
        close(progress_bar);
        
        % Find the period with the strongest signal
        [strongest_signal, best_period_idx] = max(signal_strength);
        detected_period = test_periods(best_period_idx);
        detected_depth = best_depths(best_period_idx);
        detected_duration = best_durations(best_period_idx);
        detected_epoch = best_epochs(best_period_idx);
        
        detection_snr = strongest_signal;
        
        % Package up the results
        transit_info = struct();
        transit_info.period = detected_period;
        transit_info.depth = detected_depth;
        transit_info.duration = detected_duration;
        transit_info.epoch = detected_epoch;
        transit_info.snr = detection_snr;
        transit_info.all_periods = test_periods;
        transit_info.all_powers = signal_strength;
        
        appdata.transit_results = transit_info;
        fig.UserData = appdata;
        
        % Display the results graphically
        plotPeriodogram(appdata.plot_axes.periodogram, test_periods, signal_strength, ...
                       detected_period, strongest_signal);
        
        plotPhaseFolded(appdata.plot_axes.lightcurve, time_vals, flux_vals, ...
                       detected_period, detected_epoch);
        
        % Show popup with detection details
       result_text = sprintf(['TRANSIT DETECTED!\n\n' ...
                       'Orbital Period: %.4f days\n' ...
                       'Transit Depth: %.4f (%.2f%%)\n' ...
                       'Transit Duration: %.4f days (%.2f hours)\n' ...
                       'Signal-to-Noise Ratio: %.2f\n' ...
                       'First Transit Time: %.4f days'], ...
                       detected_period, detected_depth, detected_depth*100, ...
                       detected_duration, detected_duration*24, ...
                       detection_snr, detected_epoch);

showStatus(appdata, sprintf('Transit found! Period=%.4f days, SNR=%.1f', ...
           detected_period, detection_snr), 'green');

    
catch err
    showStatus(appdata, sprintf('Detection error: %s', err.message), 'red');
    uialert(fig, err.message, 'Transit Detection Error');
end
end
function plotPeriodogram(ax, periods, powers, best_period, max_power)
% Plot the BLS power spectrum showing signal strength vs period
cla(ax);
hold(ax, 'on');
% Main periodogram line
plot(ax, periods, powers, 'b-', 'LineWidth', 1.5);

% Mark the detected period
plot(ax, best_period, max_power, 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'red');

% Show detection threshold
yline(ax, 3, 'r--', 'LineWidth', 1.5, 'Label', '3σ Detection Limit');

xlabel(ax, 'Test Period (days)');
ylabel(ax, 'Detection Power');
title(ax, sprintf('Period Search Results - Best: %.4f days', best_period));
grid(ax, 'on');
hold(ax, 'off');
end
function plotPhaseFolded(ax, time_data, flux_data, period, epoch)
% Plot all transits stacked on top of each other
% This makes the transit more visible by combining all events
% Calculate phase for each data point
orbital_phase = mod(time_data - epoch, period) / period;

% Sort by phase for cleaner plotting
[sorted_phase, sort_order] = sort(orbital_phase);
sorted_flux = flux_data(sort_order);

cla(ax);
hold(ax, 'on');

% Plot individual points
scatter(ax, sorted_phase, sorted_flux, 10, [0.3 0.3 0.8], ...
        'filled', 'MarkerFaceAlpha', 0.3);

% Also plot binned average for clarity
num_bins = 50;
bin_edges = linspace(0, 1, num_bins+1);
bin_centers = (bin_edges(1:end-1) + bin_edges(2:end)) / 2;
binned_flux = zeros(size(bin_centers));

for bin_num = 1:num_bins
    points_in_bin = sorted_phase >= bin_edges(bin_num) & ...
                    sorted_phase < bin_edges(bin_num+1);
    if sum(points_in_bin) > 0
        binned_flux(bin_num) = mean(sorted_flux(points_in_bin));
    end
end

plot(ax, bin_centers, binned_flux, 'r-', 'LineWidth', 2, 'DisplayName', 'Binned Average');

xlabel(ax, 'Orbital Phase');
ylabel(ax, 'Normalized Flux');
title(ax, sprintf('Phase-Folded Light Curve (Period = %.4f days)', period));
grid(ax, 'on');
xlim(ax, [0 1]);
legend(ax, 'Location', 'best');
hold(ax, 'off');
end
% ========================================
% PLANET PARAMETER CALCULATION
% ========================================
function calculatePlanetParameters(appdata, star_radius_field, fig)
% Calculate physical properties of the planet from the transit
% Uses Kepler's laws and basic geometry
try
    appdata = fig.UserData;
    
    if isempty(appdata.transit_results)
        uialert(fig, 'Need to detect a transit first!', 'No Transit Data');
        return;
    end
    
    showStatus(appdata, 'Calculating planet properties...', 'blue');
    
    % Get transit measurements
    orbital_period = appdata.transit_results.period;
    transit_depth = appdata.transit_results.depth;
    transit_duration = appdata.transit_results.duration;
    
    % Get stellar radius from user input (in solar radii)
    star_radius_solar = star_radius_field.Value;
    star_radius_meters = star_radius_solar * 6.96e8;  % Convert to meters
    
    % Physical constants
    grav_const = 6.67430e-11;  % G in SI units
    solar_mass = 1.989e30;     % kg
    star_mass = 1.0 * solar_mass;  % Assuming solar-mass star
    earth_radius = 6.371e6;    % meters
    jupiter_radius = 6.9911e7; % meters
    
    % Calculate planet radius from transit depth
    % Transit depth = (R_planet / R_star)^2
    radius_ratio = sqrt(transit_depth);
    planet_radius_meters = radius_ratio * star_radius_meters;
    planet_radius_earths = planet_radius_meters / earth_radius;
    planet_radius_jupiters = planet_radius_meters / jupiter_radius;
    
    % Calculate orbital distance using Kepler's 3rd law
    % P^2 = (4π^2 / GM) * a^3
    period_seconds = orbital_period * 86400;  % Convert days to seconds
    semi_major_axis = ((grav_const * star_mass * period_seconds^2) / (4 * pi^2))^(1/3);
    semi_major_axis_au = semi_major_axis / 1.496e11;  % Convert to AU
    
    % Orbital velocity
    orbit_velocity = (2 * pi * semi_major_axis) / period_seconds;
    orbit_velocity_kms = orbit_velocity / 1000;
    
    % Calculate impact parameter from transit duration
    duration_seconds = transit_duration * 86400;
    impact_param = sqrt(max(0, 1 - (duration_seconds * pi / period_seconds)^2));
    
    % Calculate orbital inclination
    cos_inclination = impact_param * star_radius_meters / semi_major_axis;
    if abs(cos_inclination) <= 1
        inclination_degrees = acosd(cos_inclination);
    else
        inclination_degrees = 90;  % Edge-on orbit
    end
    
    % Equilibrium temperature (assuming zero albedo)
    star_temp = 5778;  % Kelvin (Sun-like star)
    planet_temp = star_temp * sqrt(star_radius_meters / (2*semi_major_axis));
    
    % Insolation relative to Earth
    insolation_relative = (semi_major_axis_au)^(-2);
    
    % Store all calculated parameters
    planet_properties = struct();
    planet_properties.period = orbital_period;
    planet_properties.depth = transit_depth;
    planet_properties.duration = transit_duration;
    planet_properties.radius_earth = planet_radius_earths;
    planet_properties.radius_jupiter = planet_radius_jupiters;
    planet_properties.orbit_au = semi_major_axis_au;
    planet_properties.velocity_kms = orbit_velocity_kms;
    planet_properties.impact_param = impact_param;
    planet_properties.inclination = inclination_degrees;
    planet_properties.temperature = planet_temp;
    planet_properties.insolation = insolation_relative;
    planet_properties.star_radius = star_radius_solar;
    
    appdata.planet_params = planet_properties;
    fig.UserData = appdata;
    
    % Display results to user
    param_text = sprintf(['PLANET PROPERTIES\n\n' ...
                         '=== Transit Characteristics ===\n' ...
                         'Period: %.5f days\n' ...
                         'Depth: %.4f%% \n' ...
                         'Duration: %.3f hours\n\n' ...
                         '=== Physical Properties ===\n' ...
                         'Planet Radius: %.3f Earth radii\n' ...
                         '              (%.3f Jupiter radii)\n' ...
                         'Equilibrium Temperature: %.0f K\n' ...
                         'Insolation: %.2f × Earth\n\n' ...
                         '=== Orbital Properties ===\n' ...
                         'Semi-major Axis: %.4f AU\n' ...
                         'Orbital Velocity: %.2f km/s\n' ...
                         'Inclination: %.2f degrees\n' ...
                         'Impact Parameter: %.3f'], ...
                         orbital_period, transit_depth*100, transit_duration*24, ...
                         planet_radius_earths, planet_radius_jupiters, ...
                         planet_temp, insolation_relative, ...
                         semi_major_axis_au, orbit_velocity_kms, ...
                         inclination_degrees, impact_param);
    
    showStatus(appdata, sprintf('Calculated: R=%.2f R⊕, a=%.3f AU', ...
               planet_radius_earths, semi_major_axis_au), 'green');
    
    uialert(fig, param_text, 'Planet Parameters', 'Icon', 'info');
    
catch err
    showStatus(appdata, sprintf('Calculation error: %s', err.message), 'red');
    uialert(fig, err.message, 'Parameter Calculation Error');
end
end
% ========================================
% 3D VISUALIZATION
% ========================================
function display3DOrbit(appdata, fig)
% Create 3D visualization of the planetary system
% Shows star, planet, and orbital path
try
    appdata = fig.UserData;
    
    if isempty(appdata.planet_params)
        uialert(fig, 'Calculate parameters first!', 'No Planet Data');
        return;
    end
    
    showStatus(appdata, 'Creating 3D visualization...', 'blue');
    
    params = appdata.planet_params;
    ax = appdata.plot_axes.orbit;
    
    cla(ax);
    hold(ax, 'on');
    
    % Get system properties
    star_size = params.star_radius;
    orbit_distance = params.orbit_au;
    orbit_tilt = params.inclination;
    
    % Draw the star in the center
    [x_star, y_star, z_star] = sphere(50);
    star_plot_size = star_size * 0.00465;  % Scale for visualization
    surf(ax, x_star*star_plot_size, y_star*star_plot_size, z_star*star_plot_size, ...
         'FaceColor', [1 0.9 0.3], 'EdgeColor', 'none');
    
    % Draw the planet
    planet_size_au = params.radius_earth * 4.26e-5;
    planet_plot_size = max(planet_size_au, star_plot_size * 0.15);  % Make sure it's visible
    
    % Create orbital path
    angle_points = linspace(0, 2*pi, 100);
    orbit_x = orbit_distance * cos(angle_points);
    orbit_y = orbit_distance * sin(angle_points) * cosd(orbit_tilt);
    orbit_z = orbit_distance * sin(angle_points) * sind(orbit_tilt);
    
    % Draw orbit path
    plot3(ax, orbit_x, orbit_y, orbit_z, 'b--', 'LineWidth', 2);
    
    % Put planet at transit position
    planet_x = orbit_x(1);
    planet_y = orbit_y(1);
    planet_z = orbit_z(1);
    
    [x_planet, y_planet, z_planet] = sphere(30);
    surf(ax, x_planet*planet_plot_size + planet_x, ...
             y_planet*planet_plot_size + planet_y, ...
             z_planet*planet_plot_size + planet_z, ...
         'FaceColor', [0.3 0.5 0.8], 'EdgeColor', 'none');
    
    % Labels and formatting
    xlabel(ax, 'X (AU)');
    ylabel(ax, 'Y (AU)');
    zlabel(ax, 'Z (AU)');
    title(ax, sprintf('3D System View - Inclination: %.1f°', orbit_tilt));
    grid(ax, 'on');
    axis(ax, 'equal');
    view(ax, 45, 20);  % Nice viewing angle
    light(ax);  % Add lighting for better 3D effect
    hold(ax, 'off');
    
    % Enable interactive rotation
    rotate3d(ax, 'on');
    
    showStatus(appdata, '3D visualization ready (drag to rotate)', 'green');
    
catch err
    showStatus(appdata, sprintf('3D error: %s', err.message), 'red');
    uialert(fig, err.message, '3D Visualization Error');
end
end
% ========================================
% TRANSIT ANIMATION
% ========================================
function runTransitAnimation(appdata, fig)
% Animate the planet transiting across the star
% Shows how the transit looks and how flux changes
try
    appdata = fig.UserData;
    
    if isempty(appdata.planet_params)
        uialert(fig, 'Need planet parameters first!', 'No Parameters');
        return;
    end
    
    showStatus(appdata, 'Running animation...', 'blue');
    
    params = appdata.planet_params;
    ax = appdata.plot_axes.animation;
    
    cla(ax);
    hold(ax, 'on');
    
    % Animation settings
    total_frames = 150;
    
    % Sizes for drawing (scaled for visibility)
    star_draw_size = 1.0;
    planet_draw_size = max(params.radius_earth * 0.009, 0.08);
    
    % Orbital parameters
    orbit_radius = params.orbit_au * 215;  % Convert AU to drawing units
    orbit_angle = params.inclination;
    
    % Set up drawing area
    plot_size = max(orbit_radius * 1.3, star_draw_size * 3);
    xlim(ax, [-plot_size, plot_size]);
    ylim(ax, [-plot_size, plot_size]);
    axis(ax, 'equal');
    
    % Draw star
    [star_x, star_y] = makeCircle(0, 0, star_draw_size);
    fill(ax, star_x, star_y, [1 0.9 0.3], 'EdgeColor', 'none');
    
    % Draw initial planet position
    planet_start_x = orbit_radius;
    planet_start_y = 0;
    [planet_x, planet_y] = makeCircle(planet_start_x, planet_start_y, planet_draw_size);
    planet_shape = fill(ax, planet_x, planet_y, [0.3 0.5 0.9], 'EdgeColor', 'black');
    
    % Draw orbit path
    orbit_angles = linspace(0, 2*pi, 100);
    path_x = orbit_radius * cos(orbit_angles);
    path_y = orbit_radius * sin(orbit_angles) * cosd(orbit_angle);
    plot(ax, path_x, path_y, 'b--', 'LineWidth', 1);
    
    % Labels
    xlabel(ax, 'Distance (Stellar Radii)');
    ylabel(ax, 'Distance (Stellar Radii)');
    title(ax, 'Transit Animation');
    
    % Text display for info
    info_text = text(ax, -plot_size*0.9, plot_size*0.9, '', ...
                    'FontSize', 10, 'FontWeight', 'bold', ...
                    'BackgroundColor', 'white', 'EdgeColor', 'black');
    
    hold(ax, 'off');
    
    % Run animation loop
    frame_delay = 0.04;  % seconds between frames
    
    for frame_num = 1:total_frames
        % Calculate current position
        orbit_fraction = (frame_num-1) / total_frames;
        current_angle = orbit_fraction * 2 * pi;
        
        pos_x = orbit_radius * cos(current_angle);
        pos_y = orbit_radius * sin(current_angle) * cosd(orbit_angle);
        pos_z = orbit_radius * sin(current_angle) * sind(orbit_angle);
        
        % Update planet position
        [planet_x, planet_y] = makeCircle(pos_x, pos_y, planet_draw_size);
        set(planet_shape, 'XData', planet_x, 'YData', planet_y);
        
        % Check if planet is in front of star (transiting)
        distance_from_star = sqrt(pos_x^2 + pos_y^2);
        if pos_z < 0 && distance_from_star < star_draw_size
            % Planet is transiting - make it red
            set(planet_shape, 'FaceColor', [0.8 0.3 0.3]);
        else
            % Normal blue color
            set(planet_shape, 'FaceColor', [0.3 0.5 0.9]);
        end
        
        % Update info text
        current_time = orbit_fraction * params.period;
        set(info_text, 'String', sprintf('Time: %.3f days\nPhase: %.2f', ...
                                        current_time, orbit_fraction));
        
        drawnow;
        pause(frame_delay);
        
        % Check if window was closed
        if ~isvalid(fig)
            return;
        end
    end
    
    showStatus(appdata, 'Animation finished', 'green');
    
catch err
    showStatus(appdata, sprintf('Animation error: %s', err.message), 'red');
    if isvalid(fig)
        uialert(fig, err.message, 'Animation Error');
    end
end
end
function [x_coords, y_coords] = makeCircle(center_x, center_y, radius)
% Helper function to generate circle coordinates
angles = linspace(0, 2*pi, 50);
x_coords = center_x + radius * cos(angles);
y_coords = center_y + radius * sin(angles);
end
% ========================================
% VALIDATION AGAINST DATABASE
% ========================================
function compareWithDatabase(appdata, fig)
% Compare our detection with known exoplanets
% This helps verify if our results are accurate
try
    appdata = fig.UserData;
    
    if isempty(appdata.transit_results) || isempty(appdata.planet_params)
        uialert(fig, 'Need detection and parameters first!', 'Missing Data');
        return;
    end
    
    showStatus(appdata, 'Comparing with known planets...', 'blue');
    
    % Our detected values
    our_period = appdata.transit_results.period;
    our_depth = appdata.transit_results.depth;
    our_radius = appdata.planet_params.radius_earth;
    
    % Database of known exoplanets (from NASA Exoplanet Archive)
    % Format: Name, Period (days), Depth, Radius (Earth radii)
    known_planets = {
        'WASP-12b',    1.0914,   0.0143,   14.3;
        'HD 209458b',  3.5247,   0.0157,   13.9;
        'Kepler-10b',  0.8375,   0.000152,  1.47;
        'TRAPPIST-1e', 6.099,    0.00055,   0.92;
        '55 Cancri e', 0.7365,   0.00013,   1.99
    };
    
    num_known_planets = size(known_planets, 1);
    
    % Calculate how close we are to each known planet
    period_errors = zeros(num_known_planets, 1);
    depth_errors = zeros(num_known_planets, 1);
    radius_errors = zeros(num_known_planets, 1);
    
    for planet_idx = 1:num_known_planets
        known_period = known_planets{planet_idx, 2};
        known_depth = known_planets{planet_idx, 3};
        known_radius = known_planets{planet_idx, 4};
        
        % Calculate percent error for each parameter
        period_errors(planet_idx) = abs(our_period - known_period) / known_period * 100;
        depth_errors(planet_idx) = abs(our_depth - known_depth) / known_depth * 100;
        radius_errors(planet_idx) = abs(our_radius - known_radius) / known_radius * 100;
    end
    
    % Find best match (lowest total error)
    total_errors = period_errors + depth_errors + radius_errors;
    [best_match_error, match_idx] = min(total_errors);
    
    % Get best match details
    match_name = known_planets{match_idx, 1};
    match_period = known_planets{match_idx, 2};
    match_depth = known_planets{match_idx, 3};
    match_radius = known_planets{match_idx, 4};
    
    % Plot comparison
    ax = appdata.plot_axes.validation;
    cla(ax);
    
    param_names = {'Period (days)', 'Depth (×1000)', 'Radius (R⊕)'};
    our_values = [our_period, our_depth*1000, our_radius];
    known_values = [match_period, match_depth*1000, match_radius];
    
    bar_positions = 1:3;
    bar_width = 0.35;
    
    bar(ax, bar_positions - bar_width/2, our_values, bar_width, ...
        'FaceColor', [0.3 0.6 0.9], 'DisplayName', 'Our Detection');
    hold(ax, 'on');
    bar(ax, bar_positions + bar_width/2, known_values, bar_width, ...
        'FaceColor', [0.9 0.4 0.3], 'DisplayName', match_name);
    
    set(ax, 'XTick', bar_positions, 'XTickLabel', param_names);
    ylabel(ax, 'Value');
    title(ax, sprintf('Comparison with %s', match_name));
    legend(ax, 'Location', 'best');
    grid(ax, 'on');
    hold(ax, 'off');
    
    % Show results
    results_text = sprintf(['VALIDATION RESULTS\n\n' ...
                           'Best Match: %s\n\n' ...
                           'Period Error: %.2f%%\n' ...
                           'Depth Error: %.2f%%\n' ...
                           'Radius Error: %.2f%%\n\n' ...
                           'Total Error: %.2f%%\n\n' ...
                           'Interpretation:\n' ...
                           '< 20%% = Excellent match\n' ...
                           '20-50%% = Good match\n' ...
                           '> 50%% = Poor match'], ...
                           match_name, ...
                           period_errors(match_idx), ...
                           depth_errors(match_idx), ...
                           radius_errors(match_idx), ...
                           best_match_error);
    
    showStatus(appdata, sprintf('Best match: %s (%.1f%% error)', ...
               match_name, best_match_error), 'green');
    
    uialert(fig, results_text, 'Validation Results', 'Icon', 'info');
    
catch err
    showStatus(appdata, sprintf('Validation error: %s', err.message), 'red');
    uialert(fig, err.message, 'Validation Error');
end
end
% ========================================
% PLOTTING HELPER FUNCTION
% ========================================
function drawLightCurve(ax, time_data, flux_data, plot_title, line_color)
% Simple helper to plot light curves consistently
cla(ax);
plot(ax, time_data, flux_data, 'Color', line_color, 'LineWidth', 1);
xlabel(ax, 'Time (days)');
ylabel(ax, 'Normalized Flux');
title(ax, plot_title);
grid(ax, 'on');
end
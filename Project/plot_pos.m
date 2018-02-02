function plot_pos(t, q1, q2, plot_title)
    figure()
    hold on
    title(plot_title)
    plot(t,q1);
    plot(t,q2);
    legend('q1', 'q2');
    xlabel('Time [s]');
    ylabel('Radial position [rads]');
end

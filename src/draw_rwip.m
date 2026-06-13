function draw_rwip(ax, theta, psi, p, viz)
%DRAW_RWIP  Render one pose of the reaction-wheel inverted pendulum into axes ax.
%
%   DRAW_RWIP(ax, theta, psi, p, viz) draws the pivot, arm, and reaction wheel
%   (rim + spoke + rim marker) for a single instant.
%
%     theta : arm angle from upright [rad]  (+CCW; theta=0 is straight up)
%     psi   : ABSOLUTE wheel angle  [rad]   (= theta + phi; sets the spoke)
%     p     : params (uses p.l_w as the drawn arm length)
%     viz   : struct, .r_w = wheel display radius [m]
%
%   Screen convention: "up" is +y; a +theta (CCW) rotation tips the arm toward
%   -x. The spoke at absolute angle psi makes the wheel's spin visible.

    L = p.l_w;  r = viz.r_w;
    tip = [-L*sin(theta);  L*cos(theta)];     % wheel-centre position

    cla(ax); hold(ax,'on');
    set(ax,'Color','w');

    % ground line + pivot
    plot(ax, [-0.32 0.32], [0 0], '-', 'Color',[0.6 0.6 0.6], 'LineWidth',1);
    plot(ax, 0, 0, '^', 'MarkerSize',10, 'MarkerFaceColor',[0.3 0.3 0.3], 'MarkerEdgeColor','k');

    % arm
    plot(ax, [0 tip(1)], [0 tip(2)], '-', 'LineWidth',5, 'Color',[0.00 0.45 0.74]);

    % wheel rim
    a = linspace(0, 2*pi, 80);
    plot(ax, tip(1)+r*cos(a), tip(2)+r*sin(a), 'k', 'LineWidth',2);

    % spoke (full diameter) at absolute angle psi, + a rim marker to show spin dir
    e = r*[-sin(psi);  cos(psi)];
    plot(ax, [tip(1)-e(1) tip(1)+e(1)], [tip(2)-e(2) tip(2)+e(2)], ...
         '-', 'Color',[0.85 0.33 0.10], 'LineWidth',2);
    plot(ax, tip(1)+e(1), tip(2)+e(2), 'o', 'MarkerSize',8, ...
         'MarkerFaceColor',[0.85 0.33 0.10], 'MarkerEdgeColor','k');

    hold(ax,'off');
    axis(ax,'equal');
    xlim(ax,[-0.32 0.32]);  ylim(ax,[-0.06 0.33]);
    set(ax,'XTick',[],'YTick',[]);  box(ax,'on');
end

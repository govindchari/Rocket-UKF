function dz = eom(t,z,T,tburn,CL,CD,cg2cp,m,R,L,rho,g,vw)

    if t>tburn
        T=0;
    end

    Aside = 2*R*L;
    Anose = pi*R^2;
    I = diag([0.25*m*R^2+(1/12)*m*L^2 0.25*m*R^2+(1/12)*m*L^2 0.5*m*R^2]);
    rgp = [0;0;cg2cp];
    rd = z(4:6);
    q = z(7:10);
    w = z(11:13);
    vra = rd-vw;
    aoa = acos(dot(vra,rd)/(norm(vra)*norm(rd)));
    Aproj = Anose*cos(aoa) + Aside*sin(aoa);

    vrahat = vra/norm(vra);
    dyn_press = 0.5*rho*norm(vra)^2;

    bCi = quat2dcm(q');
    bhat = bCi' * [0;0;-1];

    % Lift, Drag, Thrust, and Gravity vectors in inertial frame
    Lvec = (dyn_press*Aproj*CL)* cross(cross(vrahat,bhat),vrahat);
    Dvec = - (dyn_press*Aproj*CD) * vrahat;
    Tvec = T * bhat;
    gvec = [0;0;g];
    Fgvec = m*gvec;

    % Attitude Kinematics
    wq = [0;w];
    qd = 0.5*quatmultiply(q',wq');
    qd = qd';
    
    % Translational and Attitude Dynamics
    rdd = m^-1 * (Fgvec + Lvec + Dvec + Tvec);
    wd = I^-1 * (cross(rgp,bCi*(Lvec+Dvec)) - cross(w,I*w));

    dz = [rd;rdd;qd;wd];
end
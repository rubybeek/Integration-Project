sine1 = dsp.SineWave(2,0.75);
sine1.SamplesPerFrame = 3000;
y = sine1();
ref = 3*y + 30;

figure
plot(ref)
var lr = ALLEX.execSuite.libRegistry;
lr.register('vektr_mathlib',
  require('./index')(
    ALLEX
  )
);

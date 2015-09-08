# -*- coding: utf-8 -*-
"""
Created on Thu Aug 20 11:02:48 2015

@author: cdondrup
"""

import xmltodict
import pybayes as pb
import numpy as np
#import time
import math
import qtc_utils as qu
import itertools as it


class QtcInitPdf(pb.Pdf):
    def __init__(self, models, rv=None):
        self._set_rv(2, rv)
        self.models = models
        self.model = 0
        self.mu = 0

    def mean(self, cond=None):
        return self.mu

    def variance(self, cond=None):
        return 1. # No clue...

    def _get_current_state_transitions(self):
        return self.models[int(self.model)][self.mean()]

    def sample(self, cond=None):
        return self.samples(1)

    def samples(self, n, cond=None):
#        st = self._get_current_state_transitions()
        # Much quicker than repeatedly calling sample like in the default implementation
        return np.zeros((2,n)).T

    def eval_log(self, x, cond=None):
        self._check_x(x)
        st = self._get_current_state_transitions()
        if not np.all([y in st for y in x]):
            return -float('inf')
        return np.log(st[np.where(st[:,0] == x[0])][0][1])


class UniIntPdf(pb.UniPdf):
    def __init__(self, num_states, a, b, rv = None):
        super(UniIntPdf, self).__init__(a, b, rv)
        self.num_states = num_states

    def sample(self, cond = None):
        return np.round(super(UniIntPdf, self).sample(cond))

    def samples(self, n, cond = None):
        # Cheating: ensuring that every state in every model has at least one particle
        # if there are enough particles. Otherwise later models will be empty. If there
        # are not enough particles, you have much bigger problems than this anyway.
        models = [np.repeat(x,self.b[0]+1) for x in np.arange(self.a[1], self.b[1]+1)]
        states = np.arange(self.a[0], self.b[0]+1)
        spam = np.array([])
        for x in models:
            spam = np.append(spam, np.append(states, x).reshape(2,-1).T)
        spam = spam.reshape(-1,2)
        ret = np.empty((n, self.shape()))
        for i in range(n):
            ret[i, :] = spam[i, :] if i < spam.shape[0] else self.sample(cond)
        return ret


class QtcPredictionPdf(pb.CPdf):
    def __init__(self, models, rv=None, cond_rv=None):
        self._set_rvs(2, rv, 2, cond_rv)
        self.models = models
        self.mu = 0

    def mean(self, cond=None):
        return self.mu

    def _set_mean(self, cond):
        self._check_cond(cond)
        self.mu = np.round(cond[0])
        return True

    def variance(self, cond=None):
        return 0. # No clue...

    def _get_current_state_transitions(self, model):
#        print int(model), self.mean()
        return self.models[int(model)][self.mean()]

    def sample(self, cond=None):
#        print "SAMPLE PARTICLE:", cond
        return self.samples(1, cond=cond)

    def samples(self, n, cond=None):
        self._set_mean(cond)
        st = self._get_current_state_transitions(cond[1])
        # Much quicker than repeatedly calling sample like in the default implementation
#        print "TRANS:", [[int(x[0]), float(x[1])] for x in st]
        try:
            r = np.append(np.random.choice(st[:,0], p=st[:,1], size=n), np.repeat(cond[1], n)).reshape(2,-1).T
        except IndexError:
#            print cond
#            print st
            return cond
#        print "SAMPLE:", cond, "->", r
        return r

    def eval_log(self, x, cond=None):
        self._check_x(x)
        self._set_mean(cond)
        st = self._get_current_state_transitions(cond[1])
        if not round(x[0]) in st:
            return -float('inf')

        # Calculating log-likelihood for multiple draws given the probabilities of the states
#        probs = [y[1] for y in st if y[0] in np.unique(x)]
#        draws = np.bincount(x)[np.unique(x)]
#        n = len(x)
#        loglike = 0.
#        for k, p in zip(draws, probs):
#            loglike += np.log(math.factorial(n)/(math.factorial(k) * math.factorial(n-k)))+n*np.log(p)+(n-k)*np.log(1-p)
#
#        return loglike

        # Not sure if this is correct, seems too trivial. However, x can only be one element then all the rest of above function is 0.
        return np.log(st[np.where(st[:,0] == x[0])][0][1])


class QtcObservationPdf(QtcPredictionPdf):
    __confusion_matrix = np.array([
        [.7, .1, .1, .1],
        [.1, .7, .1, .1],
        [.1, .1, .7, .1],
        [.1, .1, .1, .7]
    ])
#    __confusion_matrix = np.array([
#        [1., .0, .0],
#        [.0, 1., .0],
#        [.0, .0, 1.]
#    ])
    __states = [-1, 0, 1, np.nan]

    def __init__(self, qtc_states, models, rv=None, cond_rv=None):
        super(QtcObservationPdf, self).__init__(models, rv, cond_rv)
        self.qtc_states = qtc_states

    def sample(self, cond=None):
        return self.samples(1, cond=cond)

    def samples(self, n, cond=None):
        self._set_mean(cond)
        obs = self.qtc_states[int(self.mean())]
        obs_size = 2 if np.any(np.isnan(obs)) else 4
        return np.array([
            np.random.choice(
                self.__states,
                p=self.__confusion_matrix[self.__states.index(obs[x])],
                size=n)
            for x in range(obs_size)
        ]).T

    def eval_log(self, x, cond=None):
#        print "EVAL CALLED:", x, cond
        self._check_x(x)
        self._set_mean(cond)
        obs = self.qtc_states[int(x[0])]
        obs_size = 2 if np.any(np.isnan(obs)) else 4
        pred = self.qtc_states[int(self.mean())]
        if isinstance(pred, int):
            return -50.
#        print "HERE"
        pred_size = 2 if np.any(np.isnan(pred)) else 4
        length_bias = 0. #if pred_size == obs_size else -15.
        multi = 1. #if pred_size == obs_size else 2.
        size = len(obs) #min(obs_size, pred_size)
#        try:
        return np.sum(np.append([
            np.log(self.__confusion_matrix[self.__states.index(pred[y])][self.__states.index(obs[y])])*multi \
                for y in range(size)
        ], length_bias))
#        except TypeError:
#            print obs, pred, [pred[y] for y in range(size)]


class ParticleFilter(object):
    __qtc_type_lookup = {9: "qtcbs", 81: "qtccs", 91: "qtcbcs"}

    def __init__(self, paths, qtc_type=None):
        hmm = []
        for p in paths:
            with open(p) as fd:
                hmm.append(xmltodict.parse(fd.read()))
        models, qtc_states = self._create_model(hmm)

#        print "MODEL 0:", models[0][1]
#        print "MODEL 1:", models[1][1]

        x_t = pb.RVComp(2, 'x_t')
        qpp = QtcPredictionPdf(models=models, rv=x_t, cond_rv=pb.RVComp(2, 'x_tp'))
        qop = QtcObservationPdf(qtc_states=qtc_states, models=models, rv=pb.RVComp(2, 'y_t'), cond_rv=[x_t])
#        start = time.time()
#        x_tp = np.array([5,0])
#        print [qpp.sample(x_tp) for x in range(100)]
#        print "sampling elapsed:", time.time()-start
#        print qpp.eval_log(np.array([ 11., 0.]), np.array([0., 0.]))
#        print qtc_states[12]
#        print qop.samples(10, np.array([12., 0.]))
#        print qtc_states[83], qtc_states[12], qop.eval_log(np.array([83.]), np.array([12.]))
#        print qtc_states[5]
#        print qop.samples(10, np.array([5.]))
#        print qtc_states[8], qtc_states[5], qop.eval_log(np.array([8.]), np.array([5.]))

        # prepare initial particle density:
#        init_pdf = QtcInitPdf(models=[models])
        init_pdf = UniIntPdf((len(qtc_states)-1)*len(models), np.array([0., 0.]), np.array([90., 1.])) # Has to include 0 otherwise too many particles die because nothing transitions to 1 in passby
#        print "INIT:", init_pdf.samples(10)
        pf = MyParticleFilter(500, init_pdf, qpp, qop)
        print "INIT MEAN:", pf.posterior().mean()
        p = pf.emp.particles
#        print p[:,1]
        print "INIT MODEL 0: %s, 1: %s" % (np.bincount(map(int,p[:,1]))[0], np.bincount(map(int,p[:,1]))[1])
        print "INIT STATES:", np.bincount(map(int,p[:,0].flatten())), len(np.bincount(map(int,p[:,0].flatten())))
        print "INIT STATES model 0:", np.bincount(map(int,p[np.where(p[:,1] == 0),0].flatten()))
        print "INIT STATES model 1:", np.bincount(map(int,p[np.where(p[:,1] == 1),0].flatten()))

        obs_sequence = [1., 18., 27., 36., 63., 90., 9.] #[1., 18., 27., 18., 27., 36.] #[1., 10., 46., 82., 9.]#, 91.]
        for idx, obs in enumerate(obs_sequence):
            print "###############################################################"
            pf.bayes(np.array([obs, np.nan]))
            print "%s OBS:" % idx, obs
            print "%s MEAN:" % idx, pf.posterior().mean()
            p = pf.emp.particles
            try:
                print "%s MODEL 0: %s, 1: %s" % (idx, np.bincount(map(int,p[:,1]))[0], np.bincount(map(int,p[:,1]))[1])
            except:
                pass
            print "%s STATES:" % idx, np.bincount(map(int,p[:,0].flatten()))
            try:
                print "%s STATES model 0:" % idx, np.bincount(map(int,p[np.where(p[:,1] == 0),0].flatten()))
            except:
                pass
            try:
                print "%s STATES model 1:" % idx, np.bincount(map(int,p[np.where(p[:,1] == 1),0].flatten()))
            except:
                pass
            print "%s MEDIAN: %s, STATE: %s, TRANS: %s" % (idx, np.bincount(map(int,p[:,0].flatten())).argmax(), qtc_states[int(np.bincount(map(int,p[:,0].flatten())).argmax())], [[int(x[0]), float(x[1])] for x in models[0][int(np.bincount(map(int,p[:,0].flatten())).argmax())]])


#        print "INIT MEAN:", pf.posterior().mean()
#        p = pf.emp.particles
#        print "Model 0: %s, model 1: %s" % (len(p[:,1])-np.sum(p[:,1]), np.sum(p[:,1]))
#        print "INIT PARTICLES:", p
#        print "###############################################################"
#        obs = np.array([1., np.nan])
#        pf.bayes(obs)
#        print "OBS:", obs #, "EVIDENCE:", pf.evidence_log(obs)
#        print "FIRST MEAN:", pf.posterior().mean()
#        p = pf.emp.particles
#        w = pf.emp.weights
#        print "FIRST PARTICLES:", p
#        print "Model 0: %s, model 1: %s" % (len(p[:,1])-np.sum(p[:,1]), np.sum(p[:,1]))
#        print "WEIGHTS:", w
#        print "FIRST MEDIAN: %s, STATE: %s" % (np.bincount(map(int,p[:,0].flatten())).argmax(), qtc_states[int(np.bincount(map(int,p[:,0].flatten())).argmax())])
#        print int(np.round(pf.posterior().mean()[0])), qtc_states[int(np.round(pf.posterior().mean()[0]))]
#        print 1, qtc_states[1]
#        print "TRANS:", [[int(x[0]), float(x[1])] for x in models[1]]
#        print "###############################################################"
#        obs = np.array([18., np.nan])
#        pf.bayes(obs)
#        print "OBS:", obs #, "EVIDENCE:", pf.evidence_log(obs)
#        print "SECOND MEAN:", pf.posterior().mean()
#        p = pf.emp.particles
#        w = pf.emp.weights
#        print "SECOND PARTICLES:", p
#        print "Model 0: %s, model 1: %s" % (len(p[:,1])-np.sum(p[:,1]), np.sum(p[:,1]))
#        print "WEIGHTS:", w
#        print "SECOND MEDIAN: %s, STATE: %s" % (np.bincount(map(int,p[:,0].flatten())).argmax(), qtc_states[int(np.bincount(map(int,p[:,0].flatten())).argmax())])
#        print int(np.round(pf.posterior().mean()[0])), qtc_states[int(np.round(pf.posterior().mean()[0]))]
#        print "TRANS 18:", [[int(x[0]), float(x[1])] for x in models[18]]
#        print "TRANS %s:" % int(np.round(pf.posterior().mean()[0])), [[int(x[0]), float(x[1])] for x in models[int(np.round(pf.posterior().mean()[0]))]]
#        print "###############################################################"
#        obs = np.array([45., np.nan])
#        pf.bayes(obs)
#        print "OBS:", obs #, "EVIDENCE:", pf.evidence_log(obs)
#        print "THIRD MEAN:", pf.posterior().mean()
#        p = pf.emp.particles
#        w = pf.emp.weights
#        print "THIRD PARTICLES:", p
#        print "Model 0: %s, model 1: %s" % (len(p[:,1])-np.sum(p[:,1]), np.sum(p[:,1]))
#        print "WEIGHTS:", w
#        print "THIRD MEDIAN: %s, STATE: %s" % (np.bincount(map(int,p[:,0].flatten())).argmax(), qtc_states[int(np.bincount(map(int,p[:,0].flatten())).argmax())])
#        print "TRANS 45:", [[int(x[0]), float(x[1])] for x in models[45]]
#        print "###############################################################"
#        obs = np.array([72., np.nan])
#        pf.bayes(obs)
#        print "OBS:", obs #, "EVIDENCE:", pf.evidence_log(obs)
#        print "FOURTH MEAN:", pf.posterior().mean()
#        p = pf.emp.particles
#        w = pf.emp.weights
#        print "FOURTH PARTICLES:", p
#        print "Model 0: %s, model 1: %s" % (len(p[:,1])-np.sum(p[:,1]), np.sum(p[:,1]))
#        print "WEIGHTS:", w
#        print "FOURTH MEDIAN: %s, STATE: %s" % (np.bincount(map(int,p[:,0].flatten())).argmax(), qtc_states[int(np.bincount(map(int,p[:,0].flatten())).argmax())])



    def _create_model(self, hmm):
        num_symbols = max([int(x['#text']) for x in hmm[0]['mixture']['HMM']['alphabet']['symbol']])
        qtc_states = [0]
        qtc_states.extend([x for x in qu.create_states(self.__qtc_type_lookup[num_symbols])])
        qtc_states.append(num_symbols)
#        for i,q in enumerate(qtc_states):
#            print i,q
        models = []
        for h in hmm:
            model = {}
            for i in range(0,num_symbols+1):
                model[i] = np.array([np.array(map(float,[x["@target"], x['probability']])) for x in h['mixture']['HMM']['transition'] if int(x['@source']) == i and float(x['probability']) > 0.])
            model[num_symbols] = np.array([[float(num_symbols), 1.]])
            models.append(model)
        return models, qtc_states


class MyParticleFilter(pb.Filter):
    r"""Standard particle filter implementation with resampling.

    Specifying proposal density is currently unsupported, but planned; speak up if you want it!
    Posterior pdf is represented using :class:`~pybayes.pdfs.EmpPdf` and takes following form:

    .. math:: p(x_t|y_{1:t}) = \sum_{i=1}^n \omega_i \delta ( x_t - x_t^{(i)} )
    """

    def __init__(self, n, init_pdf, p_xt_xtp, p_yt_xt):
        r"""Initialise particle filter.

        :param int n: number of particles
        :param init_pdf: either :class:`~pybayes.pdfs.EmpPdf` instance that will be used
            directly as a posterior (and should already have initial particles sampled) or
            any other probability density which initial particles are sampled from
        :type init_pdf: :class:`~pybayes.pdfs.Pdf`
        :param p_xt_xtp: :math:`p(x_t|x_{t-1})` cpdf of state in *t* given state in *t-1*
        :type p_xt_xtp: :class:`~pybayes.pdfs.CPdf`
        :param p_yt_xt: :math:`p(y_t|x_t)` cpdf of observation in *t* given state in *t*
        :type p_yt_xt: :class:`~pybayes.pdfs.CPdf`
        """
        if not isinstance(n, int) or n < 1:
            raise TypeError("n must be a positive integer")
        if not isinstance(init_pdf, pb.Pdf):
            raise TypeError("init_pdf must be an instance ot the Pdf class")
        if not isinstance(p_xt_xtp, pb.CPdf) or not isinstance(p_yt_xt, pb.CPdf):
            raise TypeError("both p_xt_xtp and p_yt_xt must be instances of the CPdf class")

        dim = init_pdf.shape()  # dimension of state
        if p_xt_xtp.shape() != dim or p_xt_xtp.cond_shape() < dim:
            raise ValueError("Expected shape() and cond_shape() of p_xt_xtp will "
                + "be (respectively greater than) {0}; ({1}, {2}) given.".format(dim,
                p_xt_xtp.shape(), p_xt_xtp.cond_shape()))
        self.p_xt_xtp = p_xt_xtp
        if p_yt_xt.cond_shape() != dim:
            raise ValueError("Expected cond_shape() of p_yt_xt will be {0}; {1} given."
                .format(dim, p_yt_xt.cond_shape()))
        self.p_yt_xt = p_yt_xt

        if isinstance(init_pdf, pb.EmpPdf):
            self.emp = init_pdf  # use directly
        else:
            self.emp = pb.EmpPdf(init_pdf.samples(n))

    def bayes(self, yt, cond = None):
        r"""Perform Bayes rule for new measurement :math:`y_t`; *cond* is ignored.

        :param numpy.ndarray cond: optional condition that is passed to :math:`p(x_t|x_{t-1})`
          after :math:`x_{t-1}` so that is can be rewritten as: :math:`p(x_t|x_{t-1}, c)`.

        The algorithm is as follows:

        1. generate new particles: :math:`x_t^{(i)} = \text{sample from }
           p(x_t^{(i)}|x_{t-1}^{(i)}) \quad \forall i`
        2. recompute weights: :math:`\omega_i = p(y_t|x_t^{(i)})
           \omega_i \quad \forall i`
        3. normalise weights
        4. resample particles
        """
        bla = []
        bla0 = []
        bla1 = []
        for i in range(self.emp.particles.shape[0]):
            # generate new ith particle:
            self.emp.particles[i] = self.p_xt_xtp.sample(self.emp.particles[i])

            # recompute ith weight:
            self.emp.weights[i] *= math.exp(self.p_yt_xt.eval_log(yt, self.emp.particles[i]))

            bla.append(np.append(self.emp.particles[i], self.emp.weights[i]))
            if self.emp.particles[i][1] == 0: bla0.append(self.emp.particles[i][0])
            else: bla1.append(self.emp.particles[i][0])

        bla = np.array(bla)
        b = np.ascontiguousarray(bla).view(np.dtype((np.void, bla.dtype.itemsize * bla.shape[1])))
        _, idx = np.unique(b, return_index=True)

        print np.array([[int(x[0]), int(x[1]), int(x[2]*1000000.)] for x in bla[idx]], dtype=int)
        print "PARTICLE 0", np.bincount(bla0)
        print "PARTICLE 1", np.bincount(bla1)

        # assure that weights are normalised
        self.emp.normalise_weights()
        # resample
        self.emp.resample()
        return True

    def posterior(self):
        return self.emp
